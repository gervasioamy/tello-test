package main

import (
	"fmt"
	"image"
	"image/color"
	"io"
	"math"
	"os/exec"
	"strconv"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gobot.io/x/gobot/platforms/keyboard"
	"gocv.io/x/gocv"
)

const frameX = 800
const frameY = 600
const frameSize = frameX * frameY * 3

var green = color.RGBA{0, 255, 0, 0}
var red = color.RGBA{255, 0, 0, 0}
var black = color.RGBA{0, 0, 0, 0}

var flightData *tello.FlightData
var tracking = false
var detectSize = false
var distTolerance = 0.05 * dist(0, 0, frameX, frameY)



func main() {
	drone := tello.NewDriver("8890")
	window := gocv.NewWindow("Tello")

	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()

	proto := "./facetracking/proto.txt"
	model := "./facetracking/model"

	net := gocv.ReadNetFromCaffe(proto, model)
	if net.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", proto, model)
		return
	}
	defer net.Close()

	if net.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", proto, model)
		return
	}
	defer net.Close()

	// init values
	refDistance := float64(0)
	detected := false
	left := float32(0)
	top := float32(0)
	right := float32(0)
	bottom := float32(0)

	keys := keyboard.NewDriver()
	handleKeys(keys, drone)

	work := func() {
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.FlightDataEvent, func(data interface{}) {
			flightData = data.(*tello.FlightData)
		})

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
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
				fmt.Println(err)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		[]gobot.Device{keys},
		work,
	)

	// calling Start(false) lets the Start routine return immediately without an additional blocking goroutine
	robot.Start(false)

	// now handle video frames from ffmpeg stream in main thread, to be macOS/Windows friendly
	for {
		fmt.Println(time.Now().Format(time.StampMilli))
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}
		W := float32(img.Cols())
		H := float32(img.Rows())
		blob := gocv.BlobFromImage(img, 1.0, image.Pt(128, 96), gocv.NewScalar(104.0, 177.0, 123.0, 0), false, false)
		defer blob.Close()

		net.SetInput(blob, "data")

		detBlob := net.Forward("detection_out")
		defer detBlob.Close()

		detections := gocv.GetBlobChannel(detBlob, 0, 0)
		defer detections.Close()

		var rectToFollow image.Rectangle // if more than one face is detected, just follow the more confident
		var maxConfidence float32

		for r := 0; r < detections.Rows(); r++ {
			confidence := detections.GetFloatAt(r, 2)
			if confidence < 0.5 {
				// let's ignore those with less than 50% of confidence
				continue
			}
			fmt.Printf("Face detected [%v%%] \n", confidence*100)

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
			// green if the rec found is centered, red if it is not (drone must move)
			rect := image.Rect(int(left), int(top), int(right), int(bottom))
			createRectWithProperColor(&img, rect)

			detected = true
			if confidence > maxConfidence {
				maxConfidence = confidence
				rectToFollow = rect
			}
		}

		// Draw middle axis to know where the rectangle is
		gocv.Line(&img, image.Pt(0, frameY/2), image.Pt(frameX, frameY/2), black, 2)
		gocv.Line(&img, image.Pt(frameX/2, 0), image.Pt(frameX/2, frameY), black, 2)

		window.IMShow(img)
		if window.WaitKey(10) >= 0 {
			break
		}

		if !tracking || !detected {
			continue
		}

		if detectSize {
			detectSize = false
			// takes the initial rectangle diagonal size, as a reference to know later if
			// the drone is closer or farther.
			refDistance = dist(left, top, right, bottom)
		}

		//followFace(drone, left, top, right, bottom, refDistance)
		followFace(drone, rectToFollow, refDistance)
	}
}

func createRectWithProperColor(img *gocv.Mat, rect image.Rectangle) {
	if isRectangleCentered(rect) {
		gocv.Rectangle(img, rect, green, 3)
	} else {
		gocv.Rectangle(img, rect, red, 3)
	}
}

func isRectangleCentered(rect image.Rectangle) bool {
	return rect.Max.X > frameX/2 && rect.Max.Y > frameY/2 && rect.Min.X < frameX/2 && rect.Min.Y < frameY/2
}

/*
 It moves the drone in order to follow the face detected, if any parameter is out of the center
*/
func followFace(drone *tello.Driver, rect image.Rectangle, refDistance float64) {
//func followFace(drone *tello.Driver, left, top, right, bottom float32, refDistance float64) {
	if isRectangleCentered(rect) {
		// no need to move the drone while the rect is green
		// FIXME now it's ignoring the distance
		fmt.Println("Face tracking: HOVER drone")
		drone.Hover()
		return
	}
	// patch
	W := float32(frameX)
	H := float32(frameY)
	// ---
	left := float32(rect.Min.X)
	top := float32(rect.Min.Y)
	right := float32(rect.Max.X)
	bottom := float32(rect.Max.Y)
	actualDistance := dist(left, top, right, bottom)
	// let's see where is the face rectangle to know if the drone needs to move:
	// first the x axis:
	if right < W/2 {
		/*         W/2
		+-----------+----------+
		|     +--+  |          |
		|     |  |  |          |
		|     |  |  |          |
		|     +--+  |          |
		|           |          |
		+----------------------+
		|           |          |
		|           |          |
		|           |          |
		|           |          |
		|           |          |
		+-----------+----------+
		*/
		drone.CounterClockwise(50)
	} else if left > W/2 {
		/*         W/2
		+-----------+----------+
		|           |          |
		|           |          |
		|           |          |
		|           | +-----+  |
		|           | |     |  |
		+----------------------+
		|           | |     |  |
		|           | +-----+  |
		|           |          |
		|           |          |
		|           |          |
		+-----------+----------+
		*/
		drone.Clockwise(50)
	} else {
		/*         W/2
		+-----------+----------+
		|           |          |
		|           |          |
		|           |          |
		|           |          |
		|           |          |
		+----------------------+
		|        +-----+       |
		|        |  |  |       |
		|        |  |  |       |
		|        |  |  |       |
		|        +-----+       |
		+----------------------+
		*/
		drone.Clockwise(0)
	}
	// then, the y axis:
	if top < H/10 {
		drone.Up(25)
	} else if bottom > H-H/10 {
		drone.Down(25)
	} else {
		drone.Up(0) // implies Down = 0 as well
	}
	// and lastly, let's see if the rectangle is bigger than the reference when started face tracking
	// if so, the move backward because it means the face is close to the drone
	// if the rectangle is smaller, then the face is farther, so, move the drone forward to be closer to the face
	if actualDistance < refDistance-distTolerance {
		drone.Forward(20)
	} else if actualDistance > refDistance+distTolerance {
		drone.Backward(20)
	} else {
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
			fmt.Println(key.Char)
			drone.Clockwise(-25)
		case keyboard.ArrowRight:
			fmt.Println(key.Char)
			drone.Clockwise(25)
		case keyboard.W:
			fmt.Println(key.Char)
			drone.Forward(20)
		case keyboard.S:
			fmt.Println(key.Char)
			drone.Backward(20)
		case keyboard.A:
			fmt.Println(key.Char)
			drone.Left(20)
		case keyboard.D:
			fmt.Println(key.Char)
			drone.Right(20)
		case keyboard.ArrowDown:
			fmt.Println(key.Char)
			drone.Down(20)
		case keyboard.ArrowUp:
			fmt.Println(key.Char)
			drone.Up(20)
		case keyboard.Q:
			fmt.Println(key.Char)
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
		case keyboard.Escape:
			drone.Hover()
		case keyboard.B:
			fmt.Printf("B == Battery: %v \n", flightData.BatteryPercentage)
		case keyboard.X:
			fmt.Printf("X == STATS:\n %+v\n", flightData)
		}
	})
}

/*
 Starts / Stops the face tracking
*/
func faceTracking(drone *tello.Driver) {
	//drone.Forward(0)
	//drone.Up(0)
	//drone.Clockwise(0)
	drone.Hover()
	tracking = !tracking
	if tracking {
		detectSize = true
		fmt.Println("START face tracking")
	} else {
		detectSize = false
		fmt.Println("STOP face tracking")
	}
}