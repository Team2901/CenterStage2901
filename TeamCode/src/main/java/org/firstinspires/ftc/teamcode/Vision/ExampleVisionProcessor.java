package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class ExampleVisionProcessor implements VisionProcessor {

    Telemetry telemetry;

    Size targetSize;
    Mat outputFrameRGB = new Mat();

    Paint paint;
    Bitmap bitmap;

    public ExampleVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        targetSize = new Size(width, height);

        // Prepare some things for the Canvas onDrawFrame
        paint = new Paint();
        paint.setAntiAlias(true);
        bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
    }

    @Override
    public Object processFrame(Mat inputFrameRGB, long captureTimeNanos) {
        telemetry.addData("Capture Time", captureTimeNanos);

        if (inputFrameRGB == null) {
            telemetry.addLine("~~~ Hmm, null input frame in pipeline... ~~~");
            return null;
        }

        // Drop the alpha channel, if it exists
        if (inputFrameRGB.type() == 24) {
            Imgproc.cvtColor(inputFrameRGB, inputFrameRGB, Imgproc.COLOR_RGBA2RGB);
        }

        telemetry.update();

        // For visualization, let's show the part of the image we're seeing that's in color range
        // (First, make our mask 3 channels)
        if (true) {
            // Placeholder output frame preparation -- replace with your processing
            inputFrameRGB.copyTo(outputFrameRGB);
            return outputFrameRGB; // return value here is given back as userContext in onDrawFrame
        }
        else {
            return inputFrameRGB;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (userContext instanceof Mat) {
            // Black fill the canvas
            canvas.drawColor(Color.WHITE);

            // Cast the Mat back to a Mat
            Mat mat = (Mat)userContext;

            // Resize to canvas size (keeping aspect ratio)
            float scaleFactor = Math.min(1.0f * onscreenWidth / mat.width(), 1.0f * onscreenHeight / mat.height());
            canvas.scale(scaleFactor, scaleFactor);

            // Convert to Android Bitmap
            Utils.matToBitmap(mat, bitmap);

            // Write to the canvas
            canvas.drawBitmap(bitmap, 0, 0, paint);
        }
    }
}
