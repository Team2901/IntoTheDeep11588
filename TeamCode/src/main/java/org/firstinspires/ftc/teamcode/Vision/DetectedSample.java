package org.firstinspires.ftc.teamcode.Vision;

import androidx.annotation.NonNull;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;

import java.util.Locale;

public class DetectedSample {
    public MatOfPoint contour;
    public Point centroid;
    public Moments moments;
    public DetectedSample(MatOfPoint _contour, Point _centroid, Moments _moments){
        contour = _contour;
        centroid = _centroid;
        moments = _moments;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "c=(%.1f,%.1f), a=%.1f", centroid.x, centroid.y, moments.m00);
    }
}
