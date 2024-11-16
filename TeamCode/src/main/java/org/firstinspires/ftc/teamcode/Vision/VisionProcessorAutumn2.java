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
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class VisionProcessorAutumn2 implements VisionProcessor
{

    public static boolean doVisualization = true;

    public static Scalar hsvBlueLimitLower = new Scalar(80,80,65);
    public static Scalar hsvBlueLimitUpper = new Scalar(130,255,255);
    public static Scalar hsvRedLimitLower1 = new Scalar(0,80,65);
    public static Scalar hsvRedLimitUpper1 = new Scalar(7,255,255);
    public static Scalar hsvRedLimitLower2 = new Scalar(150,80,65);
    public static Scalar hsvRedLimitUpper2 = new Scalar(180,255,255);
    public static Scalar hsvYellowLimitLower = new Scalar(8,80,65);
    public static Scalar hsvYellowLimitUpper = new Scalar(45,255,255);
    public List<MatOfPoint> contours = new ArrayList<>();
    public List<Point> centroids = new ArrayList<>();
    Telemetry telemetry;

    int frameCount = 0;

    Size targetSize;
    Mat outputFrameRGB;
    Mat inputFrameHSV;
    Mat maskSampleBlue;
    Mat maskSampleRed1;
    Mat maskSampleRed2;
    Mat maskSampleRed;
    Mat maskSampleYellow;
    Mat outputScratchpad;
    Mat debugViewInput;
    Mat debugViewBlue;
    Mat debugViewRed;
    Mat debugViewYellow;

    public VisionProcessorAutumn2(Telemetry telemetry) {
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

    public void resize(Size size) { resize((int)size.width, (int)size.height); }

    public void resize(int width, int height) {
        synchronized (resizeLock) {
            targetSize = new Size(width, height);
            outputFrameRGB = new Mat(targetSize, CvType.CV_8UC3);
            inputFrameHSV = new Mat(targetSize, CvType.CV_8UC3);
            maskSampleBlue = new Mat(targetSize, CvType.CV_8UC1);
            maskSampleRed1 = new Mat(targetSize, CvType.CV_8UC1);
            maskSampleRed2 = new Mat(targetSize, CvType.CV_8UC1);
            maskSampleRed = new Mat(targetSize, CvType.CV_8UC1);
            maskSampleYellow = new Mat(targetSize, CvType.CV_8UC1);
            outputScratchpad = new Mat(height * 2, width * 2, CvType.CV_8UC3);
            debugViewInput = outputScratchpad.submat(0, height, 0, width);
            debugViewBlue = outputScratchpad.submat(height, height * 2, 0, width);
            debugViewRed = outputScratchpad.submat(height, height * 2, width, width * 2);
            debugViewYellow = outputScratchpad.submat(0, height, width, width * 2);

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

        //
        // Do the processing here
        //
        // Convert to HSV
        Imgproc.cvtColor(inputFrameRGB, inputFrameHSV, Imgproc.COLOR_RGB2HSV);

        // Color range
        Core.inRange(inputFrameHSV, hsvBlueLimitLower, hsvBlueLimitUpper, maskSampleBlue);
        Core.inRange(inputFrameHSV, hsvRedLimitLower1, hsvRedLimitUpper1, maskSampleRed1);
        Core.inRange(inputFrameHSV, hsvRedLimitLower2, hsvRedLimitUpper2, maskSampleRed2);
        Core.bitwise_or(maskSampleRed1, maskSampleRed2, maskSampleRed);
        Core.inRange(inputFrameHSV, hsvYellowLimitLower, hsvYellowLimitUpper, maskSampleYellow);

        Imgproc.medianBlur(maskSampleRed, maskSampleRed, 13);

        Mat hierarchy = new Mat();
        Imgproc.findContours(maskSampleRed, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        telemetry.addData("Contours 1", contours.size());

        int idx = 0;
        for (MatOfPoint contour : contours) {
            Moments moments = Imgproc.moments(contour);
            double totalPixels = moments.m00;
            double sumX = moments.m10;
            double sumY = moments.m01;
            Point centroid = new Point(sumX / totalPixels, sumY / totalPixels);
            telemetry.addData("contour" + idx + "centroid", centroid);
            centroids.add(centroid);
            idx++;
        }
        if (doVisualization) {
            // Fill/initialize the frame
            Imgproc.rectangle(outputScratchpad, new Point(0, 0), new Point(outputScratchpad.width(), outputScratchpad.height()), new Scalar(0,0,0), -1);
            double fontScale = targetSize.height / 180.0;
            Scalar fontColor = new Scalar(255,255,255);
            // For visualization, let's show the part of the image we're seeing that's in color range
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewBlue, maskSampleBlue);
            Imgproc.putText(debugViewBlue, "Blue", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewRed, maskSampleRed);
            Imgproc.putText(debugViewRed, "Red", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewYellow, maskSampleYellow);
            Imgproc.putText(debugViewYellow, "Yellow", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);
            inputFrameRGB.copyTo(debugViewInput);
            Imgproc.drawContours(debugViewRed, contours, -1, new Scalar(0, 255, 0));

            contours = new ArrayList<>();

            for(Point centroid : centroids) {
                Imgproc.circle(debugViewRed, centroid, 2, new Scalar(0, 255, 0), 2);
            }

            centroids = new ArrayList<>();

            Imgproc.line(debugViewRed,
                    new Point(debugViewRed.width() / 2, 0),
                    new Point(debugViewRed.width() / 2, debugViewRed.height()),
                    new Scalar (0, 255, 255));

            Imgproc.line(debugViewRed,
                    new Point(0, debugViewRed.height() / 2),
                    new Point(debugViewRed.width(), debugViewRed.height() / 2),
                    new Scalar (0, 255, 255));

            Imgproc.line(debugViewBlue,
                    new Point(debugViewBlue.width() / 2, 0),
                    new Point(debugViewBlue.width() / 2, debugViewBlue.height()),
                    new Scalar (0, 255, 255));

            Imgproc.line(debugViewBlue,
                    new Point(0, debugViewBlue.height() / 2),
                    new Point(debugViewBlue.width(), debugViewBlue.height() / 2),
                    new Scalar (0, 255, 255));

            Imgproc.line(debugViewYellow,
                    new Point(debugViewYellow.width() / 2, 0),
                    new Point(debugViewYellow.width() / 2, debugViewYellow.height()),
                    new Scalar (0, 255, 255));

            Imgproc.line(debugViewYellow,
                    new Point(0, debugViewYellow.height() / 2),
                    new Point(debugViewYellow.width(), debugViewYellow.height() / 2),
                    new Scalar (0, 255, 255));

            Imgproc.putText(debugViewYellow, "Yellow", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);
            inputFrameRGB.copyTo(debugViewInput);
            // (Finally, make the output frame for visualization)
            Imgproc.resize(outputScratchpad, outputFrameRGB, targetSize);
            // (Finally, make the output frame for visualization)
            Imgproc.resize(outputScratchpad, outputFrameRGB, targetSize);
        }

        telemetry.update(); // this is for EOCV-Sim
        return null; // Return value from VisionProcess is passed to onDrawFrame as userContext
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // For convenience, we can make the debug visuals in openCV code during processFrame
        // Then, here in onDrawFrame, we just draw that
        if (doVisualization) {
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
