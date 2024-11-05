package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class MyVisionProcessor implements VisionProcessor
{
    Telemetry telemetry;
    int frameCount = 0;

    Size targetSize;
    Mat outputFrameRGB = new Mat();

    public MyVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetry.setAutoClear(true); // this is for EOCV-Sim
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        targetSize = new Size(width, height);
        outputFrameRGB = new Mat(targetSize, CvType.CV_8UC3);

        // For onDrawFrame
        bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        paint = new Paint();
        paint.setAntiAlias(true);
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

        // Do the processing here
        // Converting to grayscale is just a placeholder
        Imgproc.cvtColor(inputFrameRGB, outputFrameRGB, Imgproc.COLOR_RGB2GRAY);

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

            // Resize to canvas size
            float scaleFactor = Math.min(1.0f * onscreenWidth / outputFrameRGB.width(), 1.0f * onscreenHeight / outputFrameRGB.height());
            canvas.scale(scaleFactor, scaleFactor);

            // Convert to Android Bitmap
            Utils.matToBitmap(outputFrameRGB, bitmap);

            // Write to the canvas
            canvas.drawBitmap(bitmap, 0, 0, paint);
        }
    }
    Bitmap bitmap;
    Paint paint;
}
