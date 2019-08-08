// +build example
//
// Do not build by default.

/*
You must have ffmpeg and OpenCV installed in order to run this code. It will connect to the Tello
and then open a window using OpenCV showing the streaming video.

How to run

	go run examples/tello_facetracker.go ~/Downloads/res10_300x300_ssd_iter_140000.caffemodel ~/Development/opencv/samples/dnn/face_detector/deploy.prototxt
*/

package main

import (
	"fmt"
		"image/color"
	"io"
			"os/exec"
	"strconv"
	"sync/atomic"
	"time"
	"image"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
		"gocv.io/x/gocv"
)

type pair struct {
	x float64
	y float64
}

const (
	frameX    = 960
	frameY    = 720
	frameSize = frameX * frameY * 3
	offset    = 32767.0
)

var (
	// ffmpeg command to decode video stream from drone
	ffmpeg = exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, errIn  = ffmpeg.StdinPipe()
	ffmpegOut, errOut = ffmpeg.StdoutPipe()

	// gocv
	window = gocv.NewWindow("Tello")
	net    *gocv.Net
	green  = color.RGBA{0, 255, 0, 0}

	// tracking
	tracking                 = false
	detected                 = false
	detectSize               = false
	//distTolerance            = 0.05 * dist(0, 0, frameX, frameY)
	refDistance              float64
	left, top, right, bottom float64

	// drone
	drone      = tello.NewDriver("8890")
	flightData *tello.FlightData


	leftX, leftY, rightX, rightY atomic.Value
)

func init() {
	leftX.Store(float64(0.0))
	leftY.Store(float64(0.0))
	rightX.Store(float64(0.0))
	rightY.Store(float64(0.0))

	// process drone events in separate goroutine for concurrency
	go func() {

		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.FlightDataEvent, func(data interface{}) {
			// TODO: protect flight data from race condition
			flightData = data.(*tello.FlightData)
		})

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected to tello...")
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

		robot := gobot.NewRobot("tello",
			[]gobot.Connection{},
			[]gobot.Device{drone},
		)

		robot.Start()
	}()
}

func main() {
	/*
	model := os.Args[1]
	config := os.Args[2]
	backend := gocv.NetBackendDefault
	if len(os.Args) > 3 {
		backend = gocv.ParseNetBackend(os.Args[3])
	}

	target := gocv.NetTargetCPU
	if len(os.Args) > 4 {
		target = gocv.ParseNetTarget(os.Args[4])
	}

	n := gocv.ReadNet(model, config)
	if n.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", model, config)
		return
	}
	net = &n
	defer net.Close()
	net.SetPreferableBackend(gocv.NetBackendType(backend))
	net.SetPreferableTarget(gocv.NetTargetType(target))
	*/

	for {
		// get next frame from stream
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		//img, _ := gocv.IMDecode(buf, gocv.IMReadGrayScale)
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		//img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC1, buf)
		if img.Empty() {
			fmt.Println("empty")
			continue
		}
		//trackFace(&img)


		// ===== circle tracking ===>>
		grayImg := gocv.NewMat()
		defer grayImg.Close()
		gocv.CvtColor(img, &grayImg, gocv.ColorBGRToGray)
		// img is the video frame as is (color)
		gocv.MedianBlur(grayImg, &grayImg, 5)

		//gocv.CvtColor(img, &cimg, gocv.ColorGrayToBGR)
		circles := gocv.NewMat()
		defer circles.Close()

		gocv.HoughCirclesWithParams(
			//img,
			grayImg,
			&circles,
			gocv.HoughGradient,
			1,                     // dp
			float64(img.Rows()/2), // minDist
			75,                    // param1
			20,                    // param2
			100,                    // minRadius
			0,                     // maxRadius
		)

		blue := color.RGBA{0, 0, 255, 0}
		red := color.RGBA{255, 0, 0, 0}

		for i := 0; i < circles.Cols(); i++ {
			v := circles.GetVecfAt(0, i)
			// if circles are found
			if len(v) > 2 {
				x := int(v[0])
				y := int(v[1])
				r := int(v[2])

				gocv.Circle(&img, image.Pt(x, y), r, blue, 2)
				gocv.Circle(&img, image.Pt(x, y), 2, red, 3)
			}
		}

		// ================================


		//window.IMShow(img)
		window.IMShow(grayImg)

		if window.WaitKey(10) >= 0 {
			break
		}
	}


}

/*
func trackFace(frame *gocv.Mat) {
	W := float64(frame.Cols())
	H := float64(frame.Rows())

	blob := gocv.BlobFromImage(*frame, 1.0, image.Pt(300, 300), gocv.NewScalar(104, 177, 123, 0), false, false)
	defer blob.Close()

	net.SetInput(blob, "data")

	detBlob := net.Forward("detection_out")
	defer detBlob.Close()

	detections := gocv.GetBlobChannel(detBlob, 0, 0)
	defer detections.Close()

	for r := 0; r < detections.Rows(); r++ {
		confidence := detections.GetFloatAt(r, 2)
		if confidence < 0.5 {
			continue
		}
		fmt.Printf("Index: %s - confidence: %s \n", r, confidence)

		left = float64(detections.GetFloatAt(r, 3)) * W
		top = float64(detections.GetFloatAt(r, 4)) * H
		right = float64(detections.GetFloatAt(r, 5)) * W
		bottom = float64(detections.GetFloatAt(r, 6)) * H

		left = math.Min(math.Max(0.0, left), W-1.0)
		right = math.Min(math.Max(0.0, right), W-1.0)
		bottom = math.Min(math.Max(0.0, bottom), H-1.0)
		top = math.Min(math.Max(0.0, top), H-1.0)

		detected = true
		rect := image.Rect(int(left), int(top), int(right), int(bottom))
		gocv.Rectangle(frame, rect, green, 3)
	}

	if !tracking || !detected {
		return
	}

	if detectSize {
		detectSize = false
		refDistance = dist(left, top, right, bottom)
	}

	distance := dist(left, top, right, bottom)

	// x axis
	switch {
	case right < W/2:
		drone.CounterClockwise(50)
	case left > W/2:
		drone.Clockwise(50)
	default:
		drone.Clockwise(0)
	}

	// y axis
	switch {
	case top < H/10:
		drone.Up(25)
	case bottom > H-H/10:
		drone.Down(25)
	default:
		drone.Up(0)
	}

	// z axis
	switch {
	case distance < refDistance-distTolerance:
		drone.Forward(20)
	case distance > refDistance+distTolerance:
		drone.Backward(20)
	default:
		drone.Forward(0)
	}
}

func dist(x1, y1, x2, y2 float64) float64 {
	return math.Sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
}
*/

/*
func getLeftStick() pair {
	s := pair{x: 0, y: 0}
	s.x = leftX.Load().(float64)
	s.y = leftY.Load().(float64)
	return s
}

func getRightStick() pair {
	s := pair{x: 0, y: 0}
	s.x = rightX.Load().(float64)
	s.y = rightY.Load().(float64)
	return s
}
*/