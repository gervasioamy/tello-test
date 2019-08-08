package main

import (
	"time"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"os/exec"
	"fmt"
)

//const TELLO_ID = "D88AAF"


func main2() {
	drone := tello.NewDriver("8890")
	work := func() {
		mplayer := exec.Command("mplayer", "-fps", "25", "-")
		mplayerIn, _ := mplayer.StdinPipe()
		if err := mplayer.Start(); err != nil {
			fmt.Println(err)
			return
		}
		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Connected")
			drone.StartVideo()
			drone.SetVideoEncoderRate(4)
			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})
		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := mplayerIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})
	}
	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)
	robot.Start()
}


func main() {
	drone := tello.NewDriver("8888")


	work := func() {
		drone.TakeOff()


		var ff = true;
		gobot.Every(3*time.Second, func() {
			drone.Backward(0)
			drone.Forward(0)
			time.Sleep(1 * time.Second)
			if ff {
				drone.Forward(40)
			} else {
				drone.Backward(40)
			}
			ff = !ff

		})



		gobot.After(30*time.Second, func() {
			drone.Land()
		})
	}

	/*
	work := func() {
		drone.TakeOff()
		gobot.After(3*time.Second, func() {
			drone.Forward(10)
		})
		gobot.After(6*time.Second, func() {
			drone.Backward(10)
		})
		gobot.After(9*time.Second, func() {
			drone.Land()
		})
	}
	*/

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)
	robot.Start()
}