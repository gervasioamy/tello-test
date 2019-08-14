package main

import (
	"fmt"
	"image"
	"image/color"
	"io"
	"os/exec"
	"strconv"
	"time"

	log "github.com/sirupsen/logrus"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/keyboard"
	"gocv.io/x/gocv"
	"math"
)

const (
	// Window size
	frameX = 800
	frameY = 600

	// the speed to move the drone while following a face (1-100)
	followSpeed   = 40
	keyboardSpeed = 30
)

var programRunning = true

var green = color.RGBA{0, 255, 0, 0}
var red = color.RGBA{255, 0, 0, 0}
var black = color.RGBA{0, 0, 0, 0}

var drone = tello.NewDriver("8890")

var flightData *tello.FlightData

var initialBattery int8

// flag to enable the tracking mode on the tello drone. If it's set to true then it follows the bigger face detected
var tracking = false

// rectToFollow will save the current rectangle (face) the drone should follow (if tracking mode is enabled)
// if more than one face is detected, just follow the more confident
var rectToFollow image.Rectangle

// after "start tracking" event happened, the first time a face is detected, it saves the rectangle size
// (min to max points distance) to then know the the face detection rect is bigger or smaller (to move ff or bw)
var detectedSize float64

// there's certain margin of tolerance when the rectangle of the detected face change the size, to avoid moving
// the drone unnecessarily
var distTolerance = 0.05 * dist(0, 0, frameX, frameY)

func main() {
	setupLogger()
	defer calculateUsedBattery()

	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()

	proto := "./facetracking/proto.txt"
	model := "./facetracking/model"

	window := gocv.NewWindow("Tello")
	defer window.Close()

	img := gocv.NewMat()
	defer img.Close()

	net := gocv.ReadNetFromCaffe(proto, model)
	if net.Empty() {
		log.Fatalf("Error reading network model from Proto: %v / Model: %v\n", proto, model)
		return
	}
	defer net.Close()

	keys := keyboard.NewDriver()
	handleKeys(keys, drone)

	work := func() {
		if err := ffmpeg.Start(); err != nil {
			log.Fatal("Unexpected error at ffmpeg.Start() - ", err)
			return
		}

		drone.On(tello.FlightDataEvent, func(data interface{}) {
			flightData = data.(*tello.FlightData)
			if initialBattery == 0 && flightData.BatteryPercentage > 0 {
				initialBattery = flightData.BatteryPercentage
				log.Infof("Initial battery = %v", initialBattery)
			}
		})

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			log.Info("Connected to Tello")
			drone.StartVideo()
			drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
			drone.SetExposure(0)
			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				log.Fatal(err)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		[]gobot.Device{keys},
		work,
	)

	// there are too much processing here (around 40/50 loops per second, in the "infinite" loop, than FPS, so,
	// let's send instructions to the drone every 200 ms, since the drone can't move so fast
	gobot.Every(200*time.Millisecond, func() {
		checkFollowFace()
	})

	// calling Start(false) lets the Start routine return immediately without an additional blocking goroutine
	robot.Start(false)

	defer drone.Halt()

	const frameSize = frameX * frameY * 3

	// init values
	left := float32(0)
	top := float32(0)
	right := float32(0)
	bottom := float32(0)

	// now handle video frames from ffmpeg stream in main thread, to be macOS/Windows friendly
	// until programRunning is set to false (see setupCloseHandler func)
	for programRunning {
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			log.Fatal("Error while reading from ffmpegOut", err)
			continue
		}
		img, _ = gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}
		// Draw middle axis to easy view if a rectangle (face detection) is centered or not
		gocv.Line(&img, image.Pt(0, frameY/2), image.Pt(frameX, frameY/2), black, 2)
		gocv.Line(&img, image.Pt(frameX/2, 0), image.Pt(frameX/2, frameY), black, 2)
		addTrackingText(&img)
		addAltitudeText(&img)
		addBatteryText(&img)

		W := float32(img.Cols())
		H := float32(img.Rows())

		blob := gocv.BlobFromImage(img, 1.0, image.Pt(128, 96), gocv.NewScalar(104.0, 177.0, 123.0, 0), false, false)
		net.SetInput(blob, "data")
		detBlob := net.Forward("detection_out")
		detections := gocv.GetBlobChannel(detBlob, 0, 0)

		var maxConfidence float32

		//detected := false
		rectToFollow = image.ZR // empty rectangle

		for r := 0; r < detections.Rows(); r++ {
			confidence := detections.GetFloatAt(r, 2)
			if confidence < 0.5 {
				// let's ignore those with less than 50% of confidence
				continue
			}
			log.Tracef("Face detected [%v%%] [tracking = %v]\n", int(confidence*100), tracking)

			left = detections.GetFloatAt(r, 3) * W
			top = detections.GetFloatAt(r, 4) * H
			right = detections.GetFloatAt(r, 5) * W
			bottom = detections.GetFloatAt(r, 6) * H

			// scale to video size:
			left = min(max(0, left), W-1)
			right = min(max(0, right), W-1)
			bottom = min(max(0, bottom), H-1)
			top = min(max(0, top), H-1)

			// draw a rectangle over the face recently detected
			// green if the rect found is centered, red if it is not
			rect := image.Rect(int(left), int(top), int(right), int(bottom))
			createRectWithProperColor(&img, rect)

			//detected = true
			// select the more confident detection, drone can only follow one face at a time :)
			if confidence > maxConfidence {
				maxConfidence = confidence
				rectToFollow = rect
			}
		}

		blob.Close()
		detBlob.Close()
		detections.Close()

		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}

		if !tracking {
			continue
		}

		if detectedSize == 0 {
			// takes the initial rectangle diagonal size, as a reference to know later if the drone is closer or farther
			detectedSize = dist(left, top, right, bottom)
		}
	}

}

func setupLogger() {
	log.SetFormatter(&log.TextFormatter{
		DisableColors:   false,
		FullTimestamp:   true,
		TimestampFormat: "15:04:05.000",
	})
	log.SetLevel(log.DebugLevel)
}

/*
 Draws a RED rectangle if it is not centered, or a GREEN rectangle if it is centered
*/
func createRectWithProperColor(img *gocv.Mat, rect image.Rectangle) {
	if isRectangleCentered(rect) {
		gocv.Rectangle(img, rect, green, 3)
	} else {
		gocv.Rectangle(img, rect, red, 3)
	}
}

/*
 If tracking flag is enabled and current seconds are even, it adds a text "[TRACKING]" to the window
*/
func addTrackingText(img *gocv.Mat) {
	if tracking && time.Now().Second()%2 == 0 {
		gocv.PutText(img, "[TRACKING]", image.Pt(10, 20), gocv.FontHersheyPlain, 1.2, red, 2)
	}
}

/*
 If tracking flag is enabled and current seconds are even, it adds a text "[TRACKING]" to the window
*/
func addAltitudeText(img *gocv.Mat) {
	if flightData == nil {
		return
	}
	if flightData.Flying {
		str := fmt.Sprintf("Altitude: %v m", float32(flightData.Height)/10) // Height are decimeters
		gocv.PutText(img, str, image.Pt(10, frameY-30), gocv.FontHersheyPlain, 1.2, red, 2)
	}
}

/*
 If tracking flag is enabled and current seconds are even, it adds a text "[TRACKING]" to the window
*/
func addBatteryText(img *gocv.Mat) {
	if flightData == nil {
		return
	}
	str := fmt.Sprintf("Bat.: %v%%", flightData.BatteryPercentage)
	if flightData.BatteryLower {
		gocv.PutText(img, str, image.Pt(10, frameY-60), gocv.FontHersheyPlain, 1.2, red, 3)
	} else if flightData.BatteryLow {
		gocv.PutText(img, str, image.Pt(10, frameY-60), gocv.FontHersheyPlain, 1.2, red, 2)
	} else {
		gocv.PutText(img, str, image.Pt(10, frameY-60), gocv.FontHersheyPlain, 1.2, green, 1)
	}
}

/*
 Checks if a rectangle is centered in the window
    	+----------------------+
    	|           |          |
    	|           |          |
    	|           |          |
    	|        +----+        |
    	|        |  | |        |
    	+----------------------+
    	|        |  | |        |
    	|        +----+        |
    	|           |          |
    	|           |          |
    	|           |          |
    	+----------------------+

*/
func isRectangleCentered(rect image.Rectangle) bool {
	return rect.Max.X > frameX/2 && rect.Max.Y > frameY/2 && rect.Min.X < frameX/2 && rect.Min.Y < frameY/2
}

func checkFollowFace() {
	if !tracking {
		return
	}
	followFace(drone, rectToFollow, detectedSize)
}

/*
 It moves the drone in order to follow the face detected, if any parameter
 is out of the X, Y and Z axis center
*/
func followFace(drone *tello.Driver, rect image.Rectangle, refDistance float64) {
	if rectToFollow.Empty() {
		log.Debug("[Following face] - Tracking enabled but no face detected. The drone keeps hovering\n")
		drone.Hover()
		return
	}
	followFaceX(drone, rect)
	followFaceY(drone, rect)
	followFaceZ(drone, rect, refDistance)
}

/*
 Evaluates if the face detected (rect) is out of X axis center.
 If so, moves the drone clockwise or counterclockwise.
 In the following examples, in 1) it need to rotate counterclockwise, in 2) clockwise and in 3) stop
  Ex 1: rotate counterclockwise   Ex 2 rotate clockwise         Ex 3 stop, it's centered
	+-----------+----------+       +-----------+----------+      +-----------+----------+
	|     +--+  |          |       |           |          |      |           |          |
	|     |  |  |          |       |           |          |      |           |          |
	|     |  |  |          |       |           |          |      |           |          |
	|     +--+  |          |       |           |  +-----+ |      |           |          |
	|           |          |       |           |  |     | |      |           |          |
	+----------------------+       +----------------------+      +----------------------+
	|           |          |       |           |  |     | |      |         +----+       |
	|           |          |       |           |  +-----+ |      |         | |  |       |
	|           |          |       |           |          |      |         | |  |       |
	|           |          |       |           |          |      |         | |  |       |
	|           |          |       |           |          |      |         +----+       |
	+-----------+----------+       +-----------+----------+      +----------------------+
*/
func followFaceX(drone *tello.Driver, rect image.Rectangle) {
	right := float32(rect.Max.X)
	left := float32(rect.Min.X)
	if right < frameX/2 && right > 0 {
		// Ex 1
		log.Infof("[Following face] [X] Moving CounterClockwise (right=%v)", right)
		drone.CounterClockwise(followSpeed)
	} else if left > frameX/2 && left < frameX {
		// Ex 2
		log.Infof("[Following face] [X] Moving Clockwise (left=%v)", left)
		drone.Clockwise(followSpeed)
	} else {
		// Ex 3
		log.Debugf("[Following face] [X] Stop Clockwise - [left=%v, right=%v]", left, right)
		drone.Clockwise(0)
	}
}

/*
 Evaluates if the face detected (rect) is out of Y axis center.
 If so, moves the drone up or down
 1) Up                          2) Down                        Ex 3 stop, it's centered
	+-----------+----------+       +-----------+----------+      +-----------+----------+
	|    +---+  |          |       |           |          |      |           |          |
	|    |   |  |          |       |           |          |      |           |          |
	|    |   |  |          |       |           |          |      |           |          |
	|    +---+  |          |       |           |          |      |           |   +----+ |
	|           |          |       |           |          |      |           |   |    | |
	+----------------------+       +----------------------+      +----------------------+  frameY/2
	|           |          |       |      +---+|          |      |           |   |    | |
	|           |          |       |      |   ||          |      |           |   +----+ |
	|           |          |       |      |   ||          |      |           |          |
	|           |          |       |      +---+|          |      |           |          |
	|           |          |       |           |          |      |           |          |
	+-----------+----------+       +-----------+----------+      +-----------+----------+
*/
func followFaceY(drone *tello.Driver, rect image.Rectangle) {
	bottom := float32(rect.Max.Y)
	top := float32(rect.Min.Y)
	if bottom < frameY/2 && bottom > 0 {
		log.Infof("[Following face] [Y] Moving Up (bottom=%v)", bottom)
		drone.Up(followSpeed)
	} else if top > frameY/2 && top < frameY {
		log.Infof("[Following face] [Y] Moving Down (top=%v)", top)
		drone.Down(followSpeed)
	} else {
		log.Debugf("[Following face] [Y] Stop Up/Down [top=%v, bottom=%v]\n", top, bottom)
		drone.Up(0) // implies Down = 0 as well
	}
}

/*
 Check's if the rectangle is bigger than the reference when started face tracking
 If so, moves the drone backward because it means the face is close to the drone
 If the rectangle is smaller, then the face is farther, so, it moves the drone forward to be closer to the face
*/
func followFaceZ(drone *tello.Driver, rect image.Rectangle, refDistance float64) {
	if refDistance == 0 {
		return
	}
	actualDistance := dist(float32(rect.Min.X), float32(rect.Min.Y), float32(rect.Max.X), float32(rect.Max.Y))
	if actualDistance < refDistance-distTolerance {
		log.Infof("[Following face] [Z] Moving Forward (actual:%v - refDist:%v)", int(actualDistance), int(refDistance))
		drone.Forward(30)
	} else if actualDistance > refDistance+distTolerance {
		log.Infof("[Following face] [Z] Moving Backward (actual:%v - refDist:%v)", int(actualDistance), int(refDistance))
		drone.Backward(30)
	} else {
		log.Debugf("[Following face] [Z] Stop Forward/Backward (actual:%v - refDist:%v)", int(actualDistance), int(refDistance))
		drone.Forward(0) // implies backward = 0 as well
	}
}

func dist(x1, y1, x2, y2 float32) float64 {
	return math.Sqrt(float64((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)))
}

func min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func max(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}

/*
 1 : take off
 2 : take off by throwing it up
 Q : land
 Z : palm land
 ⬅: rotate left
 ➡: rotate right
 ⬇: go down
 ⬆︎️: go up
 W : forward
 S : Backward
 A : Left
 D : Right
 T : Start / Stop face tracking
 B : Battery indicator
 X : Stats (flightData)
*/
func handleKeys(keys *keyboard.Driver, drone *tello.Driver) {
	keys.On(keyboard.Key, func(data interface{}) {
		key := data.(keyboard.KeyEvent)
		switch key.Key {
		case keyboard.ArrowLeft:
			fmt.Println("←")
			drone.CounterClockwise(keyboardSpeed)
		case keyboard.ArrowRight:
			fmt.Println("→")
			drone.Clockwise(keyboardSpeed)
		case keyboard.W:
			fmt.Println(key.Char)
			drone.Forward(keyboardSpeed)
		case keyboard.S:
			fmt.Println(key.Char)
			drone.Backward(keyboardSpeed)
		case keyboard.A:
			fmt.Println(key.Char)
			drone.Left(keyboardSpeed)
		case keyboard.D:
			fmt.Println(key.Char)
			drone.Right(keyboardSpeed)
		case keyboard.ArrowDown:
			fmt.Println("↓")
			drone.Down(keyboardSpeed)
		case keyboard.ArrowUp:
			fmt.Println("↑")
			drone.Up(keyboardSpeed)
		case keyboard.Q:
			fmt.Println(key.Char)
			tracking = false // Stop face tracking
			drone.Land()
		case keyboard.Z:
			fmt.Println(key.Char)
			drone.PalmLand()
		case keyboard.One:
			fmt.Println(key.Char)
			drone.TakeOff()
		case keyboard.Two:
			// enable take off by throwing it up
			drone.ThrowTakeOff()
		case keyboard.T:
			fmt.Println(key.Char)
			faceTracking(drone)
		case keyboard.Spacebar:
			drone.Hover()
		case keyboard.B:
			log.WithFields(log.Fields{
				"Battery":            flightData.BatteryPercentage,
				"Battery Low":        flightData.BatteryLow,
				"Battery Lower":      flightData.BatteryLower,
				"Drone Battery left": flightData.DroneBatteryLeft,
			}).Info("Battery info")
		case keyboard.X:
			fmt.Printf("X == STATS:\n %+v\n", flightData)
		case keyboard.Escape:
			log.Warn("### QUIT ###")
			programRunning = false
		}
	})
}

/*
 Starts / Stops the face tracking and stop the drone
*/
func faceTracking(drone *tello.Driver) {
	if !flightData.Flying {
		log.Warn("Face tracking is not possible if drone is not flying")
		// TODO should return here
	}
	drone.Hover()
	tracking = !tracking
	detectedSize = 0
	if tracking {
		log.Info("START Face Tracking")
	} else {
		log.Info("STOP Face Tracking")
	}
}

func calculateUsedBattery() {
	log.Infof("== CONSUMED BATTERY = %v", initialBattery-flightData.BatteryPercentage)
}
