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

public class ShapeDetection extends OpenCvPipeline {

    private Telemetry telemetry;
    public double xMidVal;
    public ShapeDetection (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat lastImage = null;
    Rect rect;

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
        Mat HSVImage = new Mat();
        Imgproc.cvtColor(lastImage, HSVImage, Imgproc.COLOR_RGB2HSV);

        Rect cropRect = new Rect(0,120,320,120);
        Imgproc.rectangle(HSVImage, cropRect, new Scalar(64, 64, 64), 10);

        Mat bwImage = new Mat();
        Core.inRange(HSVImage, new Scalar(160, 50, 70), new Scalar(180, 255, 250), bwImage);

        Mat blurImg = bwImage;
        Imgproc.medianBlur(bwImage, blurImg, 33);

        Mat cropImg = new Mat(blurImg, cropRect);

        Imgproc.findContours(blurImg, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(lastImage, contours, -1, new Scalar(0, 255, 255));

        rect = Imgproc.boundingRect(bwImage);
        Imgproc.rectangle(lastImage, rect, new Scalar(0, 255, 160), 2);

        Imgproc.line(lastImage, new Point(130,0), new Point(130,240), new Scalar(0, 0, 0));
        Imgproc.line(lastImage, new Point(280,0), new Point(280,240), new Scalar(0, 0, 0));
        if(rect != null) {
            telemetry.addData("x", rect.x);
            telemetry.addData("y", rect.y);
            telemetry.addData("xMid", this.xMid());
            xMidVal = this.xMid();
        }
        telemetry.update();

        return lastImage;
//        return bwImage;
    }

    public double xMid(){
        if(rect != null){
            return rect.x + (rect.width/2);
        }
        return 500;
    }
}
