package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShapeDetection extends OpenCvPipeline {
    // Code Review: You should combine classes ShapeDetection and ShapeDetectionBlue, and have a
    //              parameter (in constructor) for Red vs Blue. The only difference is the HSV
    //              inRange values. (And the blur kernel size is slightly different)

    private Telemetry telemetry;

    public double xMidVal;

    Size targetSize = new Size(320, 240);

    public ShapeDetection (Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Rect rect;

    public int boundingLine1 = 40;
    public int boundingLine2 = 200;

    public int spikeMark = 1;

    public int blurSize = 33;

    public Mat processFrame(Mat inputFrameRGB) {
        if (inputFrameRGB == null) {
            return null;
        }

        telemetry.clearAll();

        // Make sure the size is as expected
        // (In case we are using pixel-count thresholds)
        Imgproc.resize(inputFrameRGB, inputFrameRGB, targetSize);

        // Drop the alpha channel, if it exists
        // Note: This is only really relevant for eocv-sim and use of jpg images
        if (inputFrameRGB.type() == 24) { // CV_8U4 = 24
            Imgproc.cvtColor(inputFrameRGB, inputFrameRGB, Imgproc.COLOR_RGBA2RGB);
        }

        //convert to HSV
        Mat HSVImage = new Mat();
        Imgproc.cvtColor(inputFrameRGB, HSVImage, Imgproc.COLOR_RGB2HSV);

        // Instead of cropping, just black out the region of the frame that we want to ignore
        // Note: This is just a convenience for having an output frame to view, and not having
        //       to deal with padding back to the same size, etc.
        //Rect cropRect = new Rect(0,40,320,200);
        //Mat croppedFrame = HSVImage.submat(cropRect);
        //Mat croppedInputFrameRGB = inputFrameRGB.submat(cropRect);
        Rect blankRect = new Rect(0,0,320,40);
        Mat croppedFrame = new Mat();
        HSVImage.copyTo(croppedFrame);
        Imgproc.rectangle(croppedFrame, blankRect, new Scalar(0,0,0), -1);
        Mat croppedInputFrameRGB = new Mat();
        inputFrameRGB.copyTo(croppedInputFrameRGB);
        Imgproc.rectangle(croppedInputFrameRGB, blankRect, new Scalar(0,0,0), -1);

        // Find the pixels in the image that are our desired color (in HSV space)
        Mat bwImage = new Mat();
        Core.inRange(croppedFrame, new Scalar(160, 100, 100), new Scalar(180, 255, 250), bwImage);

        // Median blur this mask so that we can ignore the tape strips and any other noise in the mask
        Mat blurImg = new Mat();
        Imgproc.medianBlur(bwImage, blurImg, blurSize);

        // Code Review: The boundingRect is calculated on bwImage. Should it be blurImage?
        //              boundingRect will find _any_ non-zero pixel from inRange
        rect = Imgproc.boundingRect(blurImg);

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
            telemetry.addData("spikeMark", spikeMark);
        }

        // Do some visualization
        // NOTE: This can be disabled for events!
        boolean doVisualization = true;
        Mat outputFrame = null;
        if (doVisualization) {
            // Make an image that will be marked up with lines and such
            Mat markup = new Mat();
            croppedInputFrameRGB.copyTo(markup);
            Imgproc.rectangle(markup, rect, new Scalar(0, 255, 160), 2);

            // Draw the threshold lines. These should be between the tape lines separating the spikes.
            Imgproc.line(markup, new Point(boundingLine1, 0), new Point(boundingLine1, 240), new Scalar(0, 0, 0));
            Imgproc.line(markup, new Point(boundingLine2, 0), new Point(boundingLine2, 240), new Scalar(0, 0, 0));

            // Code Review: Contours are only found to draw them... Did you want to do something else
            //              with the contours? Such as use the centroid of the largest region (moment 0)?
            // For now, moved this down below the "only for visualization" line
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(blurImg, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(markup, contours, -1, new Scalar(0, 255, 255));

            // Make an image showing what the color mask output finds
            Mat onlyFoundColorMask = new Mat();
            Imgproc.cvtColor(bwImage, bwImage, Imgproc.COLOR_GRAY2RGB);
            Core.bitwise_and(bwImage, croppedInputFrameRGB, onlyFoundColorMask);

            // Make an image showing what the blur does to the color mask
            // Note: This has to be after findContours (as-is, without gray2rgb into another mat...)
            Mat onlyFoundColorBlur = new Mat();
            Imgproc.cvtColor(blurImg, blurImg, Imgproc.COLOR_GRAY2RGB);
            Core.bitwise_and(blurImg, croppedInputFrameRGB, onlyFoundColorBlur);

            // Finally, make the output frame for visualization by combining multiple images
            // Add labels
            Imgproc.putText(croppedInputFrameRGB, "Cropped input frame", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            Imgproc.putText(markup, "Marked-up", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            Imgproc.putText(onlyFoundColorMask, "bwImage mask", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            Imgproc.putText(onlyFoundColorBlur, "blurImg mask", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            // Make borders for stacking images
            Core.copyMakeBorder(inputFrameRGB, inputFrameRGB, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            Core.copyMakeBorder(croppedInputFrameRGB, croppedInputFrameRGB, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            Core.copyMakeBorder(onlyFoundColorMask, onlyFoundColorMask, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            Core.copyMakeBorder(onlyFoundColorBlur, onlyFoundColorBlur, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            Core.copyMakeBorder(markup, markup, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));

            // Make the top row
            Mat outputFrameTop = new Mat();
            Core.hconcat(Arrays.asList(onlyFoundColorMask, onlyFoundColorBlur), outputFrameTop);
            // Make the bottom row
            Mat outputFrameBottom = new Mat();
            Core.hconcat(Arrays.asList(croppedInputFrameRGB, markup), outputFrameBottom);
            // Make the output frame, combining the top and bottom rows, plus a blank space
            // I couldn't figure out how to disable the purple overlay.
            outputFrame = new Mat();
            Mat blankForOverlay = new Mat(76, outputFrameTop.width(), 16, new Scalar(102, 20, 68));
            Core.vconcat(Arrays.asList(outputFrameTop, outputFrameBottom, blankForOverlay), outputFrame);

            // EOCV-Simulator seems to flicker when image is black (0,0,0).
            // So, change it to (1,1,1)...
            Mat zeros = new Mat();
            Core.inRange(outputFrame, new Scalar(0, 0, 0), new Scalar(0, 0, 0), zeros);
            outputFrame.setTo(new Scalar(1, 1, 1), zeros);
        }

        telemetry.update();
        return outputFrame;
    }

    public double xMid(){
        if(rect != null){
            return rect.x + (rect.width/2);
        }
        return 500; // Code Review: This code does nothing. rect != null is already tested.
    }
}
