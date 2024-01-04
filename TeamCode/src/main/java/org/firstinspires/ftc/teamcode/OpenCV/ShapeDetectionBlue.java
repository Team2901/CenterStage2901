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

    private Telemetry telemetry;
    public ShapeDetectionBlue(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat lastImage = null;
    Rect rect;

    public Mat processFrame(Mat input) {
        List<MatOfPoint> blueContours = new ArrayList<>();

        telemetry.clearAll();
        if (input == null)
            return null;

        if (lastImage != null) {
            lastImage.release();
        }

        lastImage = new Mat();
        input.copyTo(lastImage);

        //convert to gray
        Mat HSVImage = new Mat();
        Imgproc.cvtColor(lastImage, HSVImage, Imgproc.COLOR_RGB2HSV);

        Rect cropRect = new Rect(0,0,320,240);
        Imgproc.rectangle(HSVImage, cropRect, new Scalar(64, 64, 64), 10);

        Mat bwImage = new Mat();
        Core.inRange(HSVImage, new Scalar(90, 55, 55), new Scalar(140, 255, 255), bwImage);

        Mat blurImg = bwImage;
        Imgproc.medianBlur(bwImage, blurImg, 19);

        Imgproc.findContours(blurImg, blueContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(lastImage, blueContours, -1, new Scalar(0, 255, 255));

        rect = Imgproc.boundingRect(bwImage);
        Imgproc.rectangle(lastImage, rect, new Scalar(0, 255, 160), 2);

        Imgproc.line(lastImage, new Point(107,0), new Point(107,240), new Scalar(0, 0, 0));
        Imgproc.line(lastImage, new Point(214,0), new Point(214,240), new Scalar(0, 0, 0));
        if(rect != null) {
            telemetry.addData("x", rect.x);
            telemetry.addData("y", rect.y);
        }
        telemetry.update();

        return lastImage;
    }

    public double xMid(){
        if(rect != null) {
            return rect.x + (rect.width/2);
        }
        return -100;
    }
}
