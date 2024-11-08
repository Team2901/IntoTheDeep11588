package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class OliviaVisionProcessor implements VisionProcessor
{
    Telemetry telemetry;
    int frameCount = 0;

    Size targetSize;
    Mat outputFrameRGB;

    public OliviaVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.setAutoClear(true); // this is for EOCV-Sim
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        resize(width, height);

        // For onDrawFrame
        paint = new Paint();
        paint.setAntiAlias(true);
    }

    public void resize(Size size) {
        resize((int)size.width, (int)size.height);
    }

    public void resize(int width, int height) {
        synchronized (resizeLock) {
            targetSize = new Size(width, height);
            outputFrameRGB = new Mat(targetSize, CvType.CV_8UC3);

            // For onDrawFrame
            bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        }
    }

    @Override
    public Object processFrame(Mat inputFrameRGB, long captureTimeNanos) {
        telemetry.addData("Capture Time", captureTimeNanos);
        telemetry.addData("Frames Processed", frameCount);
        frameCount++;

        // Drop the alpha channel, if it exists
        // This is for EOCV-Sim
        if (inputFrameRGB.type() == CvType.CV_8UC4) {
            Imgproc.cvtColor(inputFrameRGB, inputFrameRGB, Imgproc.COLOR_RGBA2RGB);
        }

        // Re-initialize size if needed
        // This is for EOCV-Sim only, so that it doesn't crash when testing an input of a different size
        if (!inputFrameRGB.size().equals(targetSize)) {
            resize(inputFrameRGB.size());
        }

        // Define what range
        Scalar lowerScalar1 = new Scalar(0, 0, 0);
        Scalar upperScalar1 = new Scalar(255, 15, 15);
        Mat inRangeFrame1 = new Mat();
        Core.inRange(inputFrameRGB, lowerScalar1, upperScalar1, inRangeFrame1);

        // convert filter from 1 channel to 3 channels
        Imgproc.cvtColor(inRangeFrame1, inRangeFrame1, Imgproc.COLOR_GRAY2RGB);

        // apply the filter to the original image
        Core.bitwise_and(inputFrameRGB, inRangeFrame1, outputFrameRGB);

        // ~~~ HSV ~~~
        // converted RGB to HSV
        Mat inputFrameHSV = new Mat();
        Imgproc.cvtColor(inputFrameRGB, inputFrameHSV, Imgproc.COLOR_RGB2HSV);

        // build an in range filter
        Scalar lowerScalar2 = new Scalar(15, 120, 0);
        Scalar upperScalar2 = new Scalar(165, 255, 255);
        Mat inRangeFrame2 = new Mat();
        Core.inRange(inputFrameHSV, lowerScalar2, upperScalar2, inRangeFrame2);

        // convert filter from 1 channel to 3 channels
        Imgproc.cvtColor(inRangeFrame2, inRangeFrame2, Imgproc.COLOR_GRAY2RGB);

        // apply the filter to the HSV image
        Mat outputFrameHsv = new Mat();
        Core.bitwise_and(inputFrameHSV, inRangeFrame2, outputFrameHsv);

        Imgproc.cvtColor(outputFrameHsv, outputFrameRGB, Imgproc.COLOR_HSV2RGB);

        //outputFrameRGB = inRangeFrame1.clone();
        // Do the processing here
        // Converting to grayscale is just a placeholder

        telemetry.update(); // this is for EOCV-Sim
        return Boolean.TRUE; // Return value from VisionProcess is passed to onDrawFrame as userContext
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // For convenience, we can make the debug visuals in openCV code during processFrame
        // Then, here in onDrawFrame, we just draw that
        if ((userContext instanceof Boolean) && ((Boolean)userContext)) {
            // Black fill the canvas
            canvas.drawColor(Color.WHITE);

            synchronized (resizeLock) {
                // Resize to canvas size
                float scaleFactor = Math.min(1.0f * onscreenWidth / outputFrameRGB.width(), 1.0f * onscreenHeight / outputFrameRGB.height());
                canvas.scale(scaleFactor, scaleFactor);

                // Convert to Android Bitmap
                Utils.matToBitmap(outputFrameRGB, bitmap);
            }

            // Write to the canvas
            canvas.drawBitmap(bitmap, 0, 0, paint);
        }
    }
    Bitmap bitmap;
    Paint paint;
    Object resizeLock = new Object();
}
