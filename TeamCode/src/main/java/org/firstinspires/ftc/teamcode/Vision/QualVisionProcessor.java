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
import java.util.Arrays;
import java.util.List;

public class QualVisionProcessor implements VisionProcessor
{
    enum SampleColor{
        RED,
        YELLOW,
        BLUE
    }
    public SampleColor interestColor = SampleColor.RED;
    public static boolean doVisualization = true;
    // Keep these!!!
    public static Scalar hsvBlueLimitLower = new Scalar(80,80,65);
    public static Scalar hsvBlueLimitUpper = new Scalar(130,255,255);
    public static Scalar hsvRedLimitLower1 = new Scalar(0,80,65);
    public static Scalar hsvRedLimitUpper1 = new Scalar(7,255,255);
    public static Scalar hsvRedLimitLower2 = new Scalar(150,80,65);
    public static Scalar hsvRedLimitUpper2 = new Scalar(180,255,255);
    // Keep these!!!
    public static Scalar hsvYellowLimitLower = new Scalar(8,80,65);
    public static Scalar hsvYellowLimitUpper = new Scalar(45,255,255);
    public List<MatOfPoint> contours = new ArrayList<>();
    public List<Point> centroids = new ArrayList<>();

    Telemetry telemetry;

    int frameCount = 0;

    public Size targetSize;
    Mat outputFrameRGB;
    Mat inputFrameHSV;
    Mat maskSampleRed1;
    Mat maskSampleRed2;
    Mat maskSample;
    Mat outputScratchpad;
    Mat debugViewInput;
    Mat bottomLeftView;
    Mat debugViewOutput;
    Mat topRightView;

    public QualVisionProcessor(Telemetry telemetry) {
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
            inputFrameHSV = new Mat(targetSize, CvType.CV_8UC3);
            maskSampleRed1 = new Mat(targetSize, CvType.CV_8UC1);
            maskSampleRed2 = new Mat(targetSize, CvType.CV_8UC1);
            maskSample = new Mat(targetSize, CvType.CV_8UC1);
            outputScratchpad = new Mat(height * 2, width * 2, CvType.CV_8UC3);
            debugViewInput = outputScratchpad.submat(0, height, 0, width);
            bottomLeftView = outputScratchpad.submat(height, height * 2, 0, width);
            debugViewOutput = outputScratchpad.submat(height, height * 2, width, width * 2);
            topRightView = outputScratchpad.submat(0, height, width, width * 2);

            // For onDrawFrame
            bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        }
    }

    @Override
    public Object processFrame(Mat inputFrameRGB, long captureTimeNanos) {
        telemetry.addData("Capture Time", captureTimeNanos);
        telemetry.addData("Frames Processed", frameCount);
        frameCount++;

        Point imageCenter = new Point();
        imageCenter.x = inputFrameRGB.width() / 2;
        imageCenter.y = inputFrameRGB.height() / 2;

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
        switch(interestColor){
            case RED:
                Core.inRange(inputFrameHSV, hsvRedLimitLower1, hsvRedLimitUpper1, maskSampleRed1);
                Core.inRange(inputFrameHSV, hsvRedLimitLower2, hsvRedLimitUpper2, maskSampleRed2);
                Core.bitwise_or(maskSampleRed1, maskSampleRed2, maskSample);
                break;
            case YELLOW:
                Core.inRange(inputFrameHSV, hsvYellowLimitLower, hsvYellowLimitUpper, maskSample);
                break;
            case BLUE:
            default:
                Core.inRange(inputFrameHSV, hsvBlueLimitLower, hsvBlueLimitUpper, maskSample);
                break;
        }
        // Median blur the mask to clean up the noise
        Imgproc.medianBlur(maskSample, maskSample, 7);

        // More processing is needed here...
        List<DetectedSample> detectedSamplesRed = findCentroidAndContours(maskSample);

        // Find the contours of our mask
        contours = new ArrayList<>();
        centroids = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(maskSample, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        telemetry.addData("Contours", contours.size());


        if (doVisualization) {

            // Fill/initialize the frame
            Imgproc.rectangle(outputScratchpad, new Point(0, 0), new Point(outputScratchpad.width(), outputScratchpad.height()), new Scalar(0,0,0), -1);

            // For visualization, let's show the part of the image we're seeing that's in color range
            inputFrameRGB.copyTo(debugViewInput);
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewOutput, maskSample);

            // Draw the contours
            // Show the centroids
            for (DetectedSample detectedSample : detectedSamplesRed) {
                Imgproc.drawContours(debugViewOutput, Arrays.asList(detectedSample.contour), -1, new Scalar(0, 255, 0));
                Imgproc.circle(debugViewOutput, detectedSample.centroid, 2, new Scalar(0, 255, 0), 2);

                int inZoneCircleRadius = 26;
                double dx = (imageCenter.x - detectedSample.centroid.x);
                double dy = (imageCenter.y - detectedSample.centroid.y);
                double d = Math.sqrt(
                        (dx * dx) + (dy * dy)
                );

                // drawing the circle in the middle - red
                if (d < inZoneCircleRadius) {
                    Imgproc.circle(debugViewOutput, detectedSample.centroid, inZoneCircleRadius, new Scalar(0, 255, 0), 2);
                } else {
                    Imgproc.circle(debugViewOutput, detectedSample.centroid, inZoneCircleRadius, new Scalar(128, 128, 128), 2);
                }
            }

            // drawing the cross lines for red, blue, yellow
            Imgproc.line(debugViewOutput,
                    new Point(debugViewOutput.width() / 2, 0),
                    new Point (debugViewOutput.width() / 2, debugViewOutput.height()),
                    new Scalar(0, 255, 255));

            Imgproc.line(debugViewOutput,
                    new Point(0, debugViewOutput.height() / 2),
                    new Point (debugViewOutput.width(), debugViewOutput.height() / 2),
                    new Scalar(0, 255, 255));
            // font stuff
            double fontScale = targetSize.height / 180.0;
            Scalar fontColor = new Scalar(255,255,255);
            Imgproc.putText(debugViewOutput, interestColor.name(), new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);

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

    public List<DetectedSample> findCentroidAndContours(Mat maskSampleColor){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(maskSampleColor, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        telemetry.addData("Contours", contours.size());

        int idx = 0;
        List<DetectedSample> detectedSamples = new ArrayList<>(contours.size());
        for (MatOfPoint contour: contours){
            // Find the center
            Moments moments = Imgproc.moments(contour);
            double totalPixels = moments.m00;
            double sumX = moments.m10;
            double sumY = moments.m01;
            Point centroid = new Point(sumX/totalPixels, sumY / totalPixels);
            DetectedSample detectedSample = new DetectedSample(contour, centroid, moments);
            telemetry.addData("contour"+idx+"centroid", centroid);
            detectedSamples.add(detectedSample);
            idx++;
        }

        return detectedSamples;
    }
    Bitmap bitmap;
    Paint paint;
    Object resizeLock = new Object();

    // Placeholder for move
    double getXPower(double Rx, double Cx,double Zx){
        double Dx = Cx - Rx;
        double m = .5;
        double Px;
        if (Math.abs(Dx) > Zx){
            Px = m * Dx - (Math.signum(Dx) * Zx);
        } else {
            Px = 0;
        }
        return Px;
    }
}