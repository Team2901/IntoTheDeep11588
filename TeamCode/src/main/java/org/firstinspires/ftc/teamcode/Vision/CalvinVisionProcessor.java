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

public class CalvinVisionProcessor implements VisionProcessor
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

    public CalvinVisionProcessor(Telemetry telemetry) {
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

        // Convert to HSV
        Imgproc.cvtColor(inputFrameRGB, inputFrameHSV, Imgproc.COLOR_RGB2HSV);

        // Color range
        Core.inRange(inputFrameHSV, hsvBlueLimitLower, hsvBlueLimitUpper, maskSampleBlue);
        Core.inRange(inputFrameHSV, hsvRedLimitLower1, hsvRedLimitUpper1, maskSampleRed1);
        Core.inRange(inputFrameHSV, hsvRedLimitLower2, hsvRedLimitUpper2, maskSampleRed2);
        Core.bitwise_or(maskSampleRed1, maskSampleRed2, maskSampleRed);
        Core.inRange(inputFrameHSV, hsvYellowLimitLower, hsvYellowLimitUpper, maskSampleYellow);

        // Median blur the mask to clean up the noise
        Imgproc.medianBlur(maskSampleRed, maskSampleRed, 7);
        Imgproc.medianBlur(maskSampleBlue, maskSampleBlue, 7);
        Imgproc.medianBlur(maskSampleYellow, maskSampleYellow, 7);

        // Get edges
        // We use the Value channel from HSV to find the edges
        Mat edges = new Mat();
        Core.extractChannel(inputFrameHSV, edges, 2);

        // We only want the part of the image which matches our color region of interest

        Core.bitwise_and(edges, maskSampleYellow, edges);

        // We will find the edges using the adaptiveThreshold method, and the inverted threshold.
        // The parameters may need to change based on resolution, lighting, testing, etc.
        // Note: Depending on how well this works for other images, we may need to use a different
        // method of edge detection
        Imgproc.adaptiveThreshold(edges, edges, 255, Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C, Imgproc.THRESH_BINARY_INV, 15, 10);
        // To clean up the threshold/binary image we will dilate erode
        // These parameters may need to change as well
        // Dilation will make the edges thicker. This will combine some edges together
        int dilationSize = 13;
        Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilationSize, dilationSize));
        Imgproc.dilate(edges, edges, dilateKernel);

        // Erosion will make the edges thinner. This will get back closer to the actual edges of the sample
        // If erosionSize is less than dilationSize, the edges will stay thicker
        int erosionSize = 13;
        Mat erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erosionSize, erosionSize));
        Imgproc.erode(edges, edges, erodeKernel);

        // More processing is needed here...
        List<DetectedSample> detectedSamplesRed = findCentroidAndContours(maskSampleRed);
        List<DetectedSample> detectedSamplesBlue = findCentroidAndContours(maskSampleBlue);
        List<DetectedSample> detectedSamplesYellow = findCentroidAndContours(edges);


        if (doVisualization) {
            // Fill/initialize the frame
            Imgproc.rectangle(outputScratchpad, new Point(0, 0), new Point(outputScratchpad.width(), outputScratchpad.height()), new Scalar(0,0,0), -1);

            // For visualization, let' show the part of the image we're seeing that's in color range
            inputFrameRGB.copyTo(debugViewInput);
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewBlue, maskSampleBlue);
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewRed, maskSampleRed);
            //Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewYellow, maskSampleYellow);
            Imgproc.cvtColor(edges, debugViewYellow, Imgproc.COLOR_GRAY2RGB);

            // Draw the contours
            // Show the centroids
            for (DetectedSample detectedSample : detectedSamplesRed){
                Imgproc.drawContours(debugViewRed, Arrays.asList(detectedSample.contour), -1, new Scalar(0, 255, 0));
                Imgproc.circle(debugViewRed, detectedSample.centroid, 2, new Scalar(0, 255, 0), 2);
            }
            for (DetectedSample detectedSample : detectedSamplesBlue){
                Imgproc.drawContours(debugViewBlue, Arrays.asList(detectedSample.contour), -1, new Scalar(0, 255, 0));
                Imgproc.circle(debugViewBlue, detectedSample.centroid, 2, new Scalar(0, 255, 0), 2);
            }
            for (DetectedSample detectedSample : detectedSamplesYellow){
                Imgproc.drawContours(debugViewYellow, Arrays.asList(detectedSample.contour), -1, new Scalar(0, 255, 0));
                Imgproc.circle(debugViewYellow, detectedSample.centroid, 2, new Scalar(0, 255, 0), 2);
            }

            // Font Stuff
            double fontScale = targetSize.height / 180.0;
            Scalar fontColor = new Scalar(255,255,255);
            Imgproc.putText(debugViewBlue, "Blue", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);
            Imgproc.putText(debugViewRed, "Red", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);
            Imgproc.putText(debugViewYellow, "Yellow", new Point(10,10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);

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
        Imgproc.findContours(maskSampleColor, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        telemetry.addData("Hierarchy", hierarchy.size());
        telemetry.addData("Contours", contours.size());
        List<Integer> firstChildren = parseHierarchyForFirstChildren(hierarchy, 0);
        telemetry.addData("First Children", firstChildren.size());
        int idx = 0;
        List<DetectedSample> detectedSamples = new ArrayList<>(firstChildren.size());
        for (int contourIndex: firstChildren){
            MatOfPoint contour = contours.get(contourIndex);
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
    // Parse the tree
// This is a recursive method. We just kick it off here with index 0.
// The return is the indices of the first-level children of the tree.
// The way findContours works, these are the biggest "holes" inside of a contour.
    private List<Integer> parseHierarchyForFirstChildren(Mat hierarchy, int index) {
        List<Integer> firstChildren = new ArrayList<>();
        if ((hierarchy == null) || (index < 0)) return firstChildren;
        double[] n = hierarchy.get(0, index);
        if (n == null) return firstChildren;
        int next = (int) n[0];
        //unused int previous = (int) n[1];
        int firstChild = (int) n[2];
        int parent = (int) n[3];
        if (parent == -1) {
            firstChildren.addAll(parseHierarchyForFirstChildren(hierarchy, firstChild));
        } else {
            firstChildren.add(index);
        }
        if (next != -1) {
            firstChildren.addAll(parseHierarchyForFirstChildren(hierarchy, next));
        }
        return firstChildren;
    }

    Bitmap bitmap;
    Paint paint;
    Object resizeLock = new Object();
}
