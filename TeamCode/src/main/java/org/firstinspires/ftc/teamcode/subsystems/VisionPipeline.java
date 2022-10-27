package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline {
    Mat greenMat;
    Mat purpleMat;
    double output1;
    double output2;
    public Rect viewRegion = new Rect (
        new Point(100, 100),
        new Point(220, 140)
    );
    Mat center1;
    Mat center2;
    @Override
    public Mat processFrame(Mat input) {
        greenMat = new Mat();
        purpleMat = new Mat();
        Imgproc.cvtColor(input, greenMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, purpleMat, Imgproc.COLOR_RGB2HSV);

        Scalar greenLow = new Scalar(50,70,80);
        Scalar greenHigh = new Scalar(80,255,255);

        Scalar purpleLow = new Scalar(30,70,80);
        Scalar purpleHigh = new Scalar(49,200,120);
        // orange (39,81,97)

        Core.inRange(greenMat, greenLow, greenHigh, greenMat);
        Core.inRange(purpleMat, purpleLow, purpleHigh, purpleMat);

        center1 = greenMat.submat(viewRegion);
        center2 = purpleMat.submat(viewRegion);

        output1 = Core.sumElems(center1).val[0] / viewRegion.area() / 255;
        output2 = Core.sumElems(center2).val[0] / viewRegion.area() / 255;

        return greenMat;
    }
    public int getLastResult() {
        double threshHold = 0.2;
        if (output1 < threshHold && output2 < threshHold) {
            if (output1 > output2) return 1;
            if (output1 < output2) return 2;
        }
        return 3;
    }
}
