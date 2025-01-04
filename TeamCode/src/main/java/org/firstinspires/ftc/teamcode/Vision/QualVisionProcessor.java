package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
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
import java.util.concurrent.atomic.AtomicReference;

@Config
public class QualVisionProcessor implements VisionProcessor , CameraStreamSource
{
    public List<TrackedSample> trackedSamples;

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        synchronized  (resizeLock) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public enum SampleColor {
        RED,
        YELLOW,
        BLUE
    }

    public static SampleColor interestColor = SampleColor.RED;

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
    public TrackedSample detectedSample;
    public static double tx = 0.5;


    Telemetry telemetry;

    int frameCount = 0;

    public static Size targetSize;
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
    Mat edges;

    public QualVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
        //telemetry.setAutoClear(true); // this is for EOCV-Sim
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        resize(width, height);

        // For onDrawFrame
        paint = new Paint();
        paint.setAntiAlias(true);
        trackedSamples = new ArrayList<TrackedSample>();
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
            edges = new Mat(targetSize, CvType.CV_8UC1);
            outputScratchpad = new Mat(height * 2, width * 2, CvType.CV_8UC3);
            debugViewInput = outputScratchpad.submat(0, height, 0, width);
            bottomLeftView = outputScratchpad.submat(height, height * 2, 0, width);
            debugViewOutput = outputScratchpad.submat(height, height * 2, width, width * 2);
            topRightView = outputScratchpad.submat(0, height, width, width * 2);

            // For onDrawFrame
            bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            lastFrame.set(bitmap);
        }
    }



    @Override
    public Object processFrame(Mat inputFrameRGB, long captureTimeNanos) {
        //telemetry.addData("Capture Time", captureTimeNanos);
        //telemetry.addData("Frames Processed", frameCount);
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
        // turning it to grayscale
        Core.extractChannel(inputFrameHSV, edges, 2);
        // only want the parts of the grayscale image that matches the color of interest
        Core.bitwise_and(edges, maskSample, edges);

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

        // detecting the samples and finding the centers
        List<DetectedSample> detectedSamples = findCentroidAndContours(edges);
        List<TrackedSample> lostSamples = new ArrayList<>();
        // Every sample seen and tracked
        for (TrackedSample trackedSample: trackedSamples){
            Double closestDistance = null;
            DetectedSample closestSample = null;
            for (DetectedSample detectedSample: detectedSamples){
                if(trackedSample.isThisMe(detectedSample)){
                    double distCentroid = trackedSample.distance(trackedSample.sample.centroid, detectedSample.centroid);
                    if (closestDistance == null || distCentroid<closestDistance){
                        closestDistance = distCentroid;
                        closestSample = detectedSample;
                    }
                }
            }
            if(closestSample != null){
                trackedSample.update(closestSample);
                detectedSamples.remove(closestSample);
            }
            else{
                // Every sample tracked but not seen
                if(trackedSample.unseen()){
                    lostSamples.add(trackedSample);
                }
            }
        }
        trackedSamples.removeAll(lostSamples);
        // Every sample seen but not tracked yet
        for (DetectedSample candidate: detectedSamples){
            TrackedSample newCandidate = new TrackedSample(candidate);
            trackedSamples.add(newCandidate);
        }

        // pick one sample from list
        // Getting the sample closest to the center
        TrackedSample bestSample = null;
        Double bestDx = null;
        for (TrackedSample candidate : trackedSamples) {
            if (!candidate.confirmed) continue;
            double cx = candidate.sample.centroid.x;
            double dx = Math.abs(cx - tx);
            if ((bestDx == null) || (dx < bestDx)) {
                bestDx = dx;
                bestSample = candidate;
            }
        }
        detectedSample = bestSample;

        //draw stuff on the screen
        if (doVisualization) {

            // Fill/initialize the frame
            Imgproc.rectangle(outputScratchpad, new Point(0, 0), new Point(outputScratchpad.width(), outputScratchpad.height()), new Scalar(0,0,0), -1);

            // For visualization, let's show the part of the image we're seeing that's in color range
            inputFrameRGB.copyTo(debugViewInput);
            Core.bitwise_and(inputFrameRGB, inputFrameRGB, debugViewOutput, maskSample);
            Imgproc.cvtColor(edges, bottomLeftView, Imgproc.COLOR_GRAY2RGB);

            // Draw the contours
            // Show the centroids
            for (TrackedSample trackedSample : trackedSamples) {
                Imgproc.drawContours(debugViewOutput, Arrays.asList(trackedSample.sample.contour), -1, new Scalar(0, 255, 0));
                Imgproc.circle(debugViewOutput, trackedSample.sample.centroid, 2, new Scalar(0, 255, 0), 2);

                int inZoneCircleRadius = 26;
                double dx = (imageCenter.x - trackedSample.sample.centroid.x);
                double dy = (imageCenter.y - trackedSample.sample.centroid.y);
                double d = Math.sqrt(
                        (dx * dx) + (dy * dy)
                );

                // drawing the circle in the middle - red
                if (d < inZoneCircleRadius) {
                    Imgproc.circle(debugViewOutput, trackedSample.sample.centroid, inZoneCircleRadius, new Scalar(0, 255, 0), 2);
                } else {
                    Imgproc.circle(debugViewOutput, trackedSample.sample.centroid, inZoneCircleRadius, new Scalar(128, 128, 128), 2);
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
            Imgproc.putText(bottomLeftView, "Edges", new Point(10, 10*fontScale), Imgproc.FONT_HERSHEY_PLAIN, fontScale, fontColor);

            // (Finally, make the output frame for visualization)
            Imgproc.resize(outputScratchpad, outputFrameRGB, targetSize);

        }

        //telemetry.update(); // this is for EOCV-Sim
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
    // Parse the tree
    // This is a recursive method. We just kick it off here with index 0.
    // The return is the indices of the first-level children of the tree.
    // The way findContours works, these are the biggest "holes" inside of a contour.
    private List<Integer> parseHierarchyForFirstChildren(Mat hierarchy, int index) {
        List<Integer> firstChildren = new ArrayList<>();
        if ((hierarchy == null) || (index < 0)) return firstChildren;
        double[] n = hierarchy.get(0, index);
        if (n == null) return firstChildren;
        int next = (int) n[0]; // gets next sibling
        //unused int previous = (int) n[1];
        int firstChild = (int) n[2];
        int parent = (int) n[3];
        if (parent == -1) {
            if (firstChild == -1){
                firstChildren.add(index);
            } else {
                firstChildren.addAll(parseHierarchyForFirstChildren(hierarchy, firstChild));
            }
        } else {
            firstChildren.add(index);
        }
        if (next != -1) {
            firstChildren.addAll(parseHierarchyForFirstChildren(hierarchy, next));
        }
        return firstChildren;
    }
    public List<DetectedSample> findCentroidAndContours(Mat edges){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //telemetry.addData("Hierarchy", hierarchy.size());
        //telemetry.addData("Contours", contours.size());
        List<Integer> firstChildren = parseHierarchyForFirstChildren(hierarchy, 0);
        hierarchy.release();
        //telemetry.addData("First Children", firstChildren.size());
        int idx = 0;
        List<DetectedSample> detectedSamples = new ArrayList<>(firstChildren.size());
        for (int contourIndex: firstChildren){
            MatOfPoint contour = contours.get(contourIndex);
            // Find the center
            Moments moments = Imgproc.moments(contour);
            double totalPixels = moments.m00;
            // 900 pixels
            // Controls edges and how far the block can be off the screen
            if (totalPixels < 1100) {
                continue;
            }
            double sumX = moments.m10;
            double sumY = moments.m01;
            Point centroid = new Point(sumX/totalPixels, sumY / totalPixels);
            DetectedSample detectedSample = new DetectedSample(contour, centroid, moments);
            //telemetry.addData("contour"+idx+"centroid", centroid);
            detectedSamples.add(detectedSample);
            idx++;
        }

        return detectedSamples;
    }
    Bitmap bitmap;
    Paint paint;
    Object resizeLock = new Object();

}