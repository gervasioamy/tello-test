package main

import (
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
)

func main() {
	drone := tello.NewDriver("8888")

	work := func() {
		drone.TakeOff()

		gobot.After(2000*time.Millisecond, func() {
			drone.Clockwise(-90)
		})
		gobot.After(2200*time.Millisecond, func() {
			drone.Clockwise(40)
		})
		gobot.After(4500*time.Millisecond, func() {
			drone.Forward(50)
		})
		gobot.After(4500*time.Millisecond, func() {
			drone.Forward(50)
		})
		gobot.After(3000*time.Millisecond, func() {
			drone.Hover()
		})

		gobot.After(6*time.Second, func() {
			drone.Land()
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)

	robot.Start()
}

