package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Arrays;

public class TrackedSample{
    private static final int UNSEEN_THRESHOLD = 1;
    private static final int SEEN_THRESHOLD = 1;
    private static final double OVERLAP_THRESHOLD = 1;
    private static final double NEAR_THRESHOLD = 1;
    public DetectedSample sample;
    public DetectedSample previousSample;
    public double nearest = 0;
    public double iou = 0;
    private Mat previousSampleMask;
    private Mat nearestSampleMask;
    private Mat intersection;
    private Mat union;
    private Rect fullFrameRect = new Rect();
    private boolean confirmed = false;
    private int hysteresisCounter = 0;
    private Scalar BLACK = new Scalar(0,0,0);

    public TrackedSample(DetectedSample candidate) {
        this.sample = candidate;
    }

    public double distance(Point p1, Point p2){
        double xDist = p2.x - p1.x;
        double yDist = p2.y - p1.y;
        double dist = Math.pow(xDist, 2) + Math.pow(yDist, 2);
        dist = Math.sqrt(dist);
        return dist;
    }

    public TrackedSample() {
        previousSampleMask = new Mat();
        nearestSampleMask = new Mat();
        intersection = new Mat();
        union = new Mat();
    }
    public boolean isThisMe(DetectedSample candidate){
        if (candidate == null) return false;

        // Calculate the distance between the centers
        nearest = distance(sample.centroid, candidate.centroid);

        // Calculate the IoU (intersection over union)
        // Black fill / clear out the previous masks
        // Note: we don't new/release in here just so that we can debug print...
        Imgproc.rectangle(previousSampleMask, fullFrameRect, BLACK, -1);
        Imgproc.rectangle(nearestSampleMask, fullFrameRect, BLACK, -1);
        // Draw filled contours; this is how we will find intersection, union, and area
        // (without having to deal with complex contour math)
        // Note: this.sample is currently the "previous frame"; this.previousSample is 2 frames ago
        Imgproc.drawContours(previousSampleMask, Arrays.asList(sample.contour), -1, new Scalar(1), -1);
        Imgproc.drawContours(nearestSampleMask, Arrays.asList(candidate.contour), -1, new Scalar(1), -1);
        // AND the masks to get the intersection
        Core.bitwise_and(previousSampleMask, nearestSampleMask, intersection);
        // OR the masks to get the union
        Core.bitwise_or(previousSampleMask, nearestSampleMask, union);
        // Compute the IoU
        iou = 1.0 * Core.countNonZero(intersection) / Core.countNonZero(union);

        // Return true if this is the sample we are tracking:
        // Overlaps enough and is near enough
        return (iou >= OVERLAP_THRESHOLD) && (nearest <= NEAR_THRESHOLD);
    }
    public boolean unseen() {
        if (confirmed) {
            // For confirmed tracks, when we dont see them
            // we count up to see if they are lost
            hysteresisCounter++;
        } else {
            // For unconfirmed tracks, when we dont see them,
            // we could down to see if they are lost
            hysteresisCounter--;
        }

        // Is this lost?
        if (confirmed && (hysteresisCounter > UNSEEN_THRESHOLD)) {
            return true;
        } else if (!confirmed && (hysteresisCounter <= 0)) {
            return true;
        } else {
            return false;
        }
    }
    public void update(DetectedSample update) {
        if (confirmed) {
            // For confirmed tracks, when found we count back down
            // to slow forgetting/losing
            hysteresisCounter--;
            if (hysteresisCounter < 0) hysteresisCounter = 0;
        } else {
            // For unconfirmed tracks, when found we count up
            // to see if we should confirm
            hysteresisCounter++;
            // Are we now confirmed?
            if (hysteresisCounter >= SEEN_THRESHOLD) {
                confirmed = true;
                // Reset the counter; now it becomes a "lost" counter
                hysteresisCounter = 0;
            }
        }

        previousSample = sample;
        sample = update;
        double alpha = 0.1;
        sample.centroid = new Point(
                alpha * update.centroid.x + (1.0 - alpha) * sample.centroid.x,
                alpha * update.centroid.y + (1.0 - alpha) * sample.centroid.y);
    }

}
