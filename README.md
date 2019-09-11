# DJI Tello test

![tello](https://product4.djicdn.com/uploads/photos/33900/medium_851441d0-f0a6-4fbc-a94a-a8fddcac149f.jpg)

This repo was created to do some PoCs to place all toghrther: [Go](https://golang.org/), [DJI Tello](https://store.dji.com/product/tello) drone, [gobot](https://gobot.io/) and [gocv](https://gocv.io/)  
  
## /facetracking
This is a simple program that just let the Tello drone detect a face and follow it automatically. You can also control the Tello drone from the keyboard
Stack:
 - OpenCV (via gocv) and caffe (for face detection)
 - [https://gobot.io/documentation/platforms/keyboard/](https://gobot.io/documentation/platforms/keyboard/)
 - [https://gobot.io/documentation/platforms/tello/](https://gobot.io/documentation/platforms/tello/)
 - [ffmpeg](https://ffmpeg.org/) for video compression

#### Set up
Before starting, you must have installed:
 - [ffmpeg](https://ffmpeg.org/download.html)
 - OpenCV: it can be installed by installing gocv [https://gocv.io/getting-started/](https://gocv.io/getting-started/)

#### How to run it?
First, connect to the drone's Wi-Fi network from your computer. It will be named something like "TELLO-XXXXXX".
Then, run the program: 
`go run ./facetracking/facetracking.go`

#### How it works?
Once program is started it, opens a window where video from the Tello drone will be streamed.

It is also listening for any key pressed on the console to control the drone:

| Key    |  Action   | 
|--------|-----------|
| 1 | take off | 
| 2 | take off by throwing it up | 
| Q | land |
| ⬅ | rotate left | 
| ➡ | rotate right |
| ⬇ | go down |
| ⬆︎️ | go up |
| W | forward |
| S | Backward |
| A | Left |
| D | Right |
| **T** | **Start / Stop face tracking** |
| B | Battery indicator| 
| X | Stats (flight data) |
| Spacebar | Hover |
| ESC | Quit Program | 

So, you need to:
- run the program
- then take off (`1` or `2`)
- put your face in front of the Tello camera (you should see a rectangle in the video window)
- then start tracking (`T`)
- Move yourself so the drone can follow you _(consider slow movements as this program is not optimized yet to support very fast movements)_
- Once you are done, then press `T` again to let the drone stop tracking

### Credits
Thanks to [@deadprogram](https://github.com/deadprogram) for the [examples provided](https://github.com/hybridgroup/gobot/tree/master/examples)
Also this [blogpost](was helpful https://tellopilots.com/threads/face-tracking-with-tello-and-gocv.374/) was helpful
