package org.firstinspires.ftc.teamcode.Vision;

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

public class ShapeDetection extends OpenCvPipeline {

    private Telemetry telemetry;
    public ShapeDetection (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat lastImage = null;

    @Override
    public Mat processFrame(Mat input) {
        List<MatOfPoint> contours = new ArrayList<>();

        telemetry.clearAll();
        if (input == null)
            return null;

        if (lastImage != null) {
            lastImage.release();
        }

        lastImage = new Mat();
        input.copyTo(lastImage);

        //convert to gray
        Mat grayImage = new Mat();
        Imgproc.cvtColor(lastImage, grayImage, Imgproc.COLOR_RGB2HSV);

        Rect cropRect = new Rect(0,0,800,600);
        Imgproc.rectangle(grayImage, cropRect, new Scalar(64, 64, 64), 10);

        Mat bwImage = new Mat();
        Core.inRange(grayImage, new Scalar(160, 50, 50), new Scalar(180, 255, 255), bwImage);

        Imgproc.findContours(bwImage, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(lastImage, contours, -1, new Scalar(0, 255, 255));

        Rect rect = Imgproc.boundingRect(bwImage);
        Imgproc.rectangle(lastImage, rect, new Scalar(0, 255, 160), 2);

        Imgproc.line(lastImage, new Point(107,0), new Point(107,240), new Scalar(0, 0, 0));
        Imgproc.line(lastImage, new Point(214,0), new Point(214,240), new Scalar(0, 0, 0));
        telemetry.addData("x", rect.x);
        telemetry.addData("y", rect.y);
        telemetry.update();

        return lastImage;
    }
}