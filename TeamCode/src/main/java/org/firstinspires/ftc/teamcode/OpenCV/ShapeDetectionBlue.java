package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ShapeDetectionBlue extends OpenCvPipeline {
    // Code Review: You should combine classes ShapeDetection and ShapeDetectionBlue, and have a
    //              parameter (in constructor) for Red vs Blue. The only difference is the HSV
    //              inRange values. (And the blur kernel size is slightly different)

    private Telemetry telemetry;
    public double xMidVal;
    public ShapeDetectionBlue(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat lastImage = null;
    Rect rect;

    int boundingLine1 = 40;
    int boundingLine2 = 200;

    public int spikeMark = 1;

    public Mat processFrame(Mat input) {
        List<MatOfPoint> blueContours = new ArrayList<>();

        // telemetry.clearAll();
        if (input == null)
            return null;

        if (lastImage != null) {
            lastImage.release();
        }

        lastImage = new Mat();
        input.copyTo(lastImage);

        //convert to HSV
        Mat HSVImage = new Mat();
        Imgproc.cvtColor(lastImage, HSVImage, Imgproc.COLOR_RGB2HSV);

        // Code Review: This doesn't crop the frame, it just draws a rectangle around the bottom
        // half of the frame
        Rect cropRect = new Rect(0,40,320,200);
        Mat croppedFrame = HSVImage.submat(cropRect);
        lastImage = lastImage.submat(cropRect);
//        Imgproc.rectangle(HSVImage, cropRect, new Scalar(64, 64, 64), 10);

        Mat bwImage = new Mat();
        Core.inRange(croppedFrame, new Scalar(80, 70, 90), new Scalar(140, 255, 255), bwImage);

        // Code Review: Note: blurImg is only used for contours... Also, this is a really big blur kernel.
        Mat blurImg = bwImage;
        Imgproc.medianBlur(bwImage, blurImg, 33);

        // Code Review: Contours are only found to draw them... Did you want to do something else
        //              with the contours? Such as use the centroid of the largest region (moment 0)?
        Imgproc.findContours(blurImg, blueContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(lastImage, blueContours, -1, new Scalar(0, 255, 255));

        // Code Review: The boundingRect is calculated on bwImage. Should it be blurImage?
        //              boundingRect will find _any_ non-zero pixel from inRange
        rect = Imgproc.boundingRect(bwImage);
        Imgproc.rectangle(lastImage, rect, new Scalar(0, 255, 160), 2);

        Imgproc.line(lastImage, new Point(boundingLine1,0), new Point(boundingLine1,240), new Scalar(0, 0, 0));
        Imgproc.line(lastImage, new Point(boundingLine2,0), new Point(boundingLine2,240), new Scalar(0, 0, 0));
        if(rect != null) {
            telemetry.addData("x", rect.x);
            telemetry.addData("y", rect.y);
            telemetry.addData("xMid", this.xMid());
            xMidVal = this.xMid();

            if (xMidVal > boundingLine2) {
                spikeMark = 3;
            } else if (xMidVal > boundingLine1) {
                spikeMark = 2;
            } else {
                spikeMark = 1;
            }
        }
        telemetry.update();

        return lastImage;
//        return bwImage;
    }

    public double xMid(){
        if(rect != null) {
            return rect.x + (rect.width/2);
        }
        return 500; // Code Review: This code does nothing. rect != null is already tested.
    }
}
