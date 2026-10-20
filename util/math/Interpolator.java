package org.firstinspires.ftc.teamcode.livewire.util.math;

import org.opencv.core.Point;

import java.util.ArrayList;

public class Interpolator {
    private ArrayList<Point> points;

    private Point leftPoint;
    private Point rightPoint;

    public Interpolator(ArrayList<Point> points) {
        this.points = points;
    }
    private int getNearestPointIndex(double x) {
        if (x < points.get(0).x) {
            return 0;
        }

        for (int i = 0; i < points.size(); i ++) {
            if (points.get(i).x > x) {
               return i - 1;
            }
        }
        // return the second to last point instead
        return points.size() - 2;
    }

    // Returns the interpolated number
    public double get(double x) {

        this.leftPoint = points.get(getNearestPointIndex(x));
        this.rightPoint = points.get(getNearestPointIndex(x) + 1);

        // calculate the line formula between the two points
        double slope = (rightPoint.y - leftPoint.y) / (rightPoint.x - leftPoint.x);
        double yIntercept = (leftPoint.y - (slope * leftPoint.x));

        return (slope * x) + yIntercept;
    }
}
