package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.CombinedHardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class ShapeDetection extends OpenCvPipeline {
    private Telemetry telemetry;
    private CombinedHardware robot;

    // To be deleted - kept only for old code
    public double xMidVal = 0;

    public Double pixelXValAverage;
    public static final double newPixelXValWeight = 0.2;

    public static final boolean INCLUDE_MORE_RED_RANGE = false;

    Size targetSize = new Size(320, 240);
    Size pipelineSize;

    // Private things, allocate once
    private static final int CV_8U3 = 16;
    private static final int CV_8U4 = 24;
    private static final Rect blankRect = new Rect(0, 0, 320, 40);
    private Mat threeChannelInputFrame = null;
    private final Mat resizedInputFrame = new Mat(targetSize, CV_8U3);
    private final Mat HSVImage = new Mat(targetSize, CV_8U3);
    private final Mat croppedFrame = new Mat(targetSize, CV_8U3);
    private final Mat croppedInputFrameRGB = new Mat(targetSize, CV_8U3);
    private final Mat bwImage = new Mat(targetSize, CV_8U3);
    private final Mat blurImg = new Mat(targetSize, CV_8U3);
    private final Mat markup = new Mat(targetSize, CV_8U3);
    private final Mat onlyFoundColorMask = new Mat(targetSize, CV_8U3);
    private final Mat onlyFoundColorBlur = new Mat(targetSize, CV_8U3);
    private final Mat outputFrameTop = new Mat(targetSize, CV_8U3);
    private final Mat outputFrameBottom = new Mat(targetSize, CV_8U3);
    private Mat outputFrame = null;
    private Mat blankForOverlay = null;
    private Mat zeros = new Mat();

    // Public configuration
    public static int blurSize = 21;
    public static boolean doVisualization = true;
    public static boolean usingCentroid = true;

    public int spikeMark = 1; // TODO: Make this an Enum

    public ShapeDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public ShapeDetection(Telemetry telemetry, CombinedHardware robot) {
        this.telemetry = telemetry;
        this.robot = robot;
    }

    @Override
    public void init(Mat firstFrame) {
        pipelineSize = firstFrame.size();
        threeChannelInputFrame = new Mat(pipelineSize, CV_8U3);
        outputFrame = new Mat(pipelineSize, CV_8U3);
    }

    public Mat processFrame(Mat inputFrameRGB) {
        if (inputFrameRGB == null) {
            return null;
        }

        // Drop the alpha channel, if it exists
        // Note: This is only really relevant for eocv-sim and use of jpg images
        if (inputFrameRGB.type() == CV_8U4) {
            Imgproc.cvtColor(inputFrameRGB, threeChannelInputFrame, Imgproc.COLOR_RGBA2RGB);
        } else {
            inputFrameRGB.copyTo(threeChannelInputFrame);
        }

        // Make sure the size is as expected
        // (In case we are using pixel-count thresholds)
        Imgproc.resize(threeChannelInputFrame, resizedInputFrame, targetSize);

        //convert to HSV
        Imgproc.cvtColor(resizedInputFrame, HSVImage, Imgproc.COLOR_RGB2HSV);

        // Instead of cropping, just black out the region of the frame that we want to ignore
        // Note: This is just a convenience for having an output frame to view, and not having
        //       to deal with padding back to the same size, etc.
        //Rect cropRect = new Rect(0,40,320,200);
        //Mat croppedFrame = HSVImage.submat(cropRect);
        //Mat croppedInputFrameRGB = inputFrameRGB.submat(cropRect);
        HSVImage.copyTo(croppedFrame);
        Imgproc.rectangle(croppedFrame, blankRect, new Scalar(0, 0, 0), -1);
        resizedInputFrame.copyTo(croppedInputFrameRGB);
        Imgproc.rectangle(croppedInputFrameRGB, blankRect, new Scalar(0, 0, 0), -1);

        // Find the pixels in the image that are our desired color (in HSV space)
        if (robot == null || robot.alliance == CombinedHardware.Alliance.RED) {
            Core.inRange(croppedFrame, new Scalar(160, 90, 80), new Scalar(180, 255, 255), bwImage);
            if(INCLUDE_MORE_RED_RANGE) {
                Mat bwImage2 = bwImage.clone();
                Core.inRange(croppedFrame, new Scalar(0, 90, 80), new Scalar(25, 90, 80), bwImage2);
                Core.bitwise_or(bwImage, bwImage2, bwImage);
            }
            //red-orange color ranges from 0-25, how to incorporate??
        } else {
            Core.inRange(croppedFrame, new Scalar(80, 70, 90), new Scalar(140, 255, 255), bwImage);
        }

        // Median blur this mask so that we can ignore the tape strips and any other noise in the mask
        Imgproc.medianBlur(bwImage, blurImg, blurSize);

        // Get the bounding rectangle from the mask after median blur
        Rect rect = Imgproc.boundingRect(blurImg);

        if (usingCentroid) {
            Mat eroded = new Mat();
            Mat strElement = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(7, 7));
            Imgproc.erode(blurImg, eroded, strElement);
            Mat dilated = new Mat();
            Imgproc.dilate(eroded, dilated, strElement);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(dilated, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.drawContours(markup, contours, -1, new Scalar(0, 255, 255));

            // Centroid of contours
            if (contours.size() > 0) {
                Moments moments = Imgproc.moments(contours.get(0));
                double totalPixels = moments.m00;
                double sumX = moments.m10;
                double sumY = moments.m01;
                double averageX = sumX / totalPixels;
                double averageY = sumY / totalPixels;
                Imgproc.circle(markup, new Point(averageX, averageY), 2, new Scalar(0, 50, 70), 2);

                telemetry.addData("Average X", averageX);
                telemetry.addData("Average Y", averageY);
                telemetry.addData("Sum X", sumX);
                telemetry.addData("Sum Y", sumY);
                telemetry.addData("Total Pixels", totalPixels);

                addPixelXVal(averageX);
            } else {
                telemetry.addLine("No Contours Found");
                return inputFrameRGB;
            }
        } else {
            // Determine the middle of the bounding rect, in x dimension
            telemetry.addData("x", rect.x);
            telemetry.addData("y", rect.y);
            xMidVal = rect.x + (rect.width / 2.0);
            telemetry.addData("X Mid", xMidVal);

            // Classify the spikemark based on the x mid value
            if(!rect.empty())addPixelXVal(xMidVal);
        }

        // Do some visualization
        // NOTE: This can be disabled for events!
        if (doVisualization) {
            // Make an image that will be marked up with lines and such
            croppedInputFrameRGB.copyTo(markup);
            Imgproc.rectangle(markup, rect, new Scalar(0, 255, 160), 2);

            // Draw the threshold lines. These should be between the tape lines separating the spikes.
            Imgproc.line(markup, new Point(robot.boundingLine1, 0), new Point(robot.boundingLine1, 240), new Scalar(0, 0, 0));
            Imgproc.line(markup, new Point(robot.boundingLine2, 0), new Point(robot.boundingLine2, 240), new Scalar(0, 0, 0));

            // Make an image showing what the color mask output finds
            Imgproc.cvtColor(bwImage, bwImage, Imgproc.COLOR_GRAY2RGB);
            Core.bitwise_and(bwImage, croppedInputFrameRGB, onlyFoundColorMask);

            // Make an image showing what the blur does to the color mask
            // Note: This has to be after findContours (as-is, without gray2rgb into another mat...)
            Imgproc.cvtColor(blurImg, blurImg, Imgproc.COLOR_GRAY2RGB);
            Core.bitwise_and(blurImg, croppedInputFrameRGB, onlyFoundColorBlur);

            // Finally, make the output frame for visualization by combining multiple images
            // Add labels
            Imgproc.putText(croppedInputFrameRGB, "Cropped input frame", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            Imgproc.putText(markup, "Marked-up", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            Imgproc.putText(onlyFoundColorMask, "bwImage mask", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            Imgproc.putText(onlyFoundColorBlur, "blurImg mask", new Point(5, 20), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255, 255, 0));
            // Make borders for stacking images
            //Core.copyMakeBorder(resizedInputFrame, resizedInputFrame, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            //Core.copyMakeBorder(croppedInputFrameRGB, croppedInputFrameRGB, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            //Core.copyMakeBorder(onlyFoundColorMask, onlyFoundColorMask, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            //Core.copyMakeBorder(onlyFoundColorBlur, onlyFoundColorBlur, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));
            //Core.copyMakeBorder(markup, markup, 1, 1, 1, 1, Core.BORDER_CONSTANT, new Scalar(255, 255, 255));

            // Make the top row
            Core.hconcat(Arrays.asList(onlyFoundColorMask, onlyFoundColorBlur), outputFrameTop);
            // Make the bottom row
            Core.hconcat(Arrays.asList(croppedInputFrameRGB, markup), outputFrameBottom);
            // Make the output frame, combining the top and bottom rows, plus a blank space
            // I couldn't figure out how to disable the purple overlay.
            if (blankForOverlay == null) {
                blankForOverlay = new Mat(76, outputFrameTop.width(), CV_8U3, new Scalar(102, 20, 68));
            }
            Core.vconcat(Arrays.asList(outputFrameTop, outputFrameBottom, blankForOverlay), outputFrame);

            // EOCV-Simulator seems to flicker when image is black (0,0,0).
            // So, change it to (1,1,1)...
            Core.inRange(outputFrame, new Scalar(0, 0, 0), new Scalar(0, 0, 0), zeros);
            outputFrame.setTo(new Scalar(1, 1, 1), zeros);

            Imgproc.resize(outputFrame, outputFrame, pipelineSize);
        } else {
            inputFrameRGB.copyTo(outputFrame);
        }

        telemetry.update();
        return outputFrame;
    }

    public void addPixelXVal(double pixelXVal) {
        if (pixelXValAverage == null) {
            pixelXValAverage = pixelXVal;
        } else {
            pixelXValAverage = (pixelXValAverage * (1 - newPixelXValWeight) + pixelXVal * newPixelXValWeight);
        }

        if (pixelXValAverage > robot.boundingLine2) {
            spikeMark = 3;
        } else if (pixelXValAverage > robot.boundingLine1) {
            spikeMark = 2;
        } else {
            spikeMark = 1;
        }
        telemetry.addData("Spike Mark", spikeMark);
    }
}
