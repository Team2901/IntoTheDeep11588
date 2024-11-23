package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;

public class DetectedSample {
    public MatOfPoint contour;
    public Point centroid;
    public Moments moments;
    public DetectedSample(MatOfPoint _contour, Point _centroid, Moments _moments){
        contour = _contour;
        centroid = _centroid;
        moments = _moments;
    }

    public String toString() {
        return centroid.toString() + ", " + moments.m00;
    }
}
