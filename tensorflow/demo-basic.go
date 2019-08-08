package main

import (
	"gocv.io/x/gocv"
	"image"
	"fmt"
)

func main() {

	webcam, _ := gocv.VideoCaptureDevice(0)
	img := gocv.NewMat()
	net := gocv.ReadNetFromTensorflow("./models/frozen_inference_graph.pb")

	for {
		// read image from camera
		webcam.Read(&img)

		// convert to a 224x244 image blob that can be processed by Tensorflow
		blob := gocv.BlobFromImage(img, 1.0, image.Pt(224, 244), gocv.NewScalar(0, 0, 0, 0), true, false)
		defer blob.Close()

		// feed the blob into the classifier
		net.SetInput(blob, "input")

		// run a forward pass thru the network
		prob := net.Forward("softmax2")
		defer prob.Close()

		// reshape the results into a 1x1000 matrix
		probMat := prob.Reshape(1, 1)
		defer probMat.Close()

		// determine the most probable classification, and display it
		_, maxVal, _, maxLoc := gocv.MinMaxLoc(probMat)
		fmt.Printf("maxLoc: %v, maxVal: %v\n", maxLoc, maxVal)

		gocv.WaitKey(1)
	}
}
