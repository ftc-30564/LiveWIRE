package org.firstinspires.ftc.teamcode.livewire.util.math.units;

import java.util.Optional;

public class Line2d {
    // uses standard form of a line. ax + by = c
    private double a;
    private double b;
    private double c;

    public Line2d(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    /**
     * slope intercept form.
     * @param slope
     * @param yIntercept
     */
    public Line2d(double slope, double yIntercept) {
        this.a = -slope;
        this.b = 1;
        this.c = yIntercept;
    }

    /**
     * point point form
     */
    public Line2d(Vector2d vector) {
        // vertical line
        if (vector.x == 0) {
            this.a = 1;
            this.b = 0;
            this.c = 0;
            return;
        }

        double slope = vector.y / vector.x;
        this.a = -slope;
        this.b = 1;
        this.c = vector.y;
    }

    /**
     * point point form
     * @param point1
     * @param point2
     */
    public Line2d(Vector2d point1, Vector2d point2) {
        // vertical line
        if (point1.x == point2.x) {
            this.a = 1;
            this.b = 0;
            this.c = point1.x;
            return;
        }

        double slope = point2.minus(point1).getSlope();
        this.a = -slope;
        this.b = 1;
        this.c = (-slope * point1.x) + point1.y;
    }

    /**
     * point slope form to standard form.
     * y = m(x - x1) + y1
     * y = mx - mx1 + y1
     * -mx + y = -mx1 + y1
     */
    public Line2d(Vector2d point, double slope) {
        this.a = -slope;
        this.b = 1;
        this.c = (-slope * point.x) + point.y;
    }

    public Optional<Double> slope() {
        // if it's a vertical line
        if (this.b == 0) {
            return Optional.empty();
        }
        return Optional.of(-this.a / this.b);
    }

    // negative reciprocal
    public Optional<Double> perpendicularSlope() {
        if (this.slope().isPresent()) {
            if (this.slope().get() == 0) {
                return Optional.empty();
            }
            return Optional.of(1 / this.slope().get());
        }
        // must be a vertical line
        return Optional.of(0.0);
    }

    // Returns the point of intersection from a given line
    public Vector2d pointOfIntersection(Line2d line) {
        // this uses Cramer's rule (todo: read up on this)
        double det = this.a * line.b - line.a * this.b;

        if (det == 0) {
            return null;
        }

        double x = (this.c * line.b - line.c * this.b) / det;
        double y = (this.a * line.c - line.a * this.c) / det;
        return new Vector2d(x, y);
    }

    // returns the closest (perpendicular) distance to a point
    public double distanceToPoint(Vector2d point) {
        Line2d perpendicularLine = new Line2d(this.b, -this.a, (point.x*this.b - point.y*this.a));

        return point.distanceFrom(pointOfIntersection(perpendicularLine));
    }
}
