package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
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

    private int centerX = -100;

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

        Rect cropRect = new Rect(0,0,800,600);
        Imgproc.rectangle(HSVImage, cropRect, new Scalar(64, 64, 64), 10);

        Mat bwImage = new Mat();
        Core.inRange(HSVImage, new Scalar(85, 30, 50), new Scalar(135, 255, 255), bwImage);

        Mat blurImg = bwImage;
        Imgproc.medianBlur(bwImage, blurImg, 5);

        Imgproc.findContours(blurImg, blueContours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        if(blueContours.size() > 0) {//prevent errors from 0 or 1 points
            Imgproc.drawContours(lastImage, blueContours, -1, new Scalar(0, 255, 255));
            rect = Imgproc.boundingRect(blurImg);
            Imgproc.rectangle(lastImage, rect, new Scalar(0, 255, 160), 2);
            MatOfPoint biggestMoP = blueContours.get(0);
            for(int i = 0; i < blueContours.size();i++){
                MatOfPoint mom = blueContours.get(i);
                if(Imgproc.contourArea(mom) > Imgproc.contourArea(biggestMoP)){
                    biggestMoP = mom;
                }
            }
            Moments moment = Imgproc.moments(biggestMoP);
            int cx = (int)(moment.get_m10()/moment.get_m00());
            int cy = (int)(moment.get_m01()/moment.get_m00());
            Imgproc.circle(lastImage, new Point(cx,cy), 6, new Scalar(255,126,255),-1);
            centerX = cx;
        }else {
            rect = new Rect();
        }

        Imgproc.line(lastImage, new Point(107,0), new Point(107,240), new Scalar(0, 0, 0));
        Imgproc.line(lastImage, new Point(214,0), new Point(214,240), new Scalar(0, 0, 0));

        telemetry.update();

        return lastImage;
    }

    public double xCoord(){
        return centerX;
    }
}
