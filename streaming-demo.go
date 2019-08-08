package main

import (
	"fmt"     // Formatted I/O
	"io"      //  It provides basic interfaces to I/O primitives
	"os/exec" // To run the external commands.
	"strconv" // Package strconv implements conversions to and from string
	"time"    //For time related operation

	"gobot.io/x/gobot"                     // Gobot Framework.
	"gobot.io/x/gobot/platforms/dji/tello" // DJI Tello package.
	"gocv.io/x/gocv"                       // GoCV package to access the OpenCV library.
)

// Frame size constant.
const (
	frameX    = 960
	frameY    = 720
	frameSize = frameX * frameY * 3
)

func main() {
	// Driver: Tello Driver
	drone := tello.NewDriver("8890")

	// OpenCV window to watch the live video stream from Tello.
	window := gocv.NewWindow("Tello")

	//FFMPEG command to convert the raw video from the drone.
	ffmpeg := exec.Command("ffmpeg", "-hwaccel", "auto", "-hwaccel_device", "opencl", "-i", "pipe:0",
		"-pix_fmt", "bgr24", "-s", strconv.Itoa(frameX)+"x"+strconv.Itoa(frameY), "-f", "rawvideo", "pipe:1")
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()

	fmt.Println(ffmpeg)

	work := func() {
		//Starting FFMPEG.
		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}
		// Event: Listening the Tello connect event to start the video streaming.
		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected to Tello.")
			if err := drone.StartVideo(); err != nil {
				fmt.Printf("StartVideo failed == %s\n", err)
			}
			drone.SetVideoEncoderRate(tello.VideoBitRateAuto)
			fmt.Println("SetVideoEncoderRate ok")
			drone.SetExposure(0)

			//For continued streaming of video.
			gobot.Every(100*time.Millisecond, func() {
				if err := drone.StartVideo(); err != nil {
					fmt.Printf("StartVideo failed == %s\n", err)
				}
			})
		})

		//Event: Piping the video data into the FFMPEG function.
		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			fmt.Println("VideoFrameEvent")
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})

		//TakeOff the Drone.
		gobot.After(5*time.Second, func() {
			//drone.TakeOff()
			fmt.Println("Tello Taking Off...")
		})

		//Land the Drone.
		gobot.After(15*time.Second, func() {
			//drone.Land()
			fmt.Println("Tello Landing...")
		})
	}
	//Robot: Tello Drone
	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)

	// calling Start(false) lets the Start routine return immediately without an additional blocking goroutine
	robot.Start(false)

	// now handle video frames from ffmpeg stream in main thread, to be macOs friendly
	for {
		fmt.Println("en el for 1")
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}
		fmt.Println("en el for 2")
		img, _ := gocv.NewMatFromBytes(frameY, frameX, gocv.MatTypeCV8UC3, buf)
		if img.Empty() {
			continue
		}
		fmt.Println("en el for 3")
		window.IMShow(img)
		if window.WaitKey(1) >= 0 {
			break
		}
	}
}
