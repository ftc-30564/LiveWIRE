package org.firstinspires.ftc.teamcode.util.math.units;

import android.app.ActivityOptions;

public class Vector2d {
    public double x;
    public double y;

    public Vector2d() {
        this(0, 0);
    }

    /**
     * Constructor for Vector2d
     * @param x The x value of the vector
     * @param y The y value of the vector
     */
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Adds two vectors
     * @param add Vector to add to this vector
     * @return The resulting vector
     */
    public Vector2d plus(Vector2d add) {
        return new Vector2d(this.x + add.x, this.y + add.y);
    }

    /**
     * Subtracts two vectors
     * @param sub Vector to subtract from this vector
     * @return The resulting vector
     */
    public Vector2d minus(Vector2d sub) {
        return new Vector2d(this.x - sub.x, this.y - sub.y);
    }

    /**
     * Calculates the magnitude of the vector using the Pythagorean Theorem.
     *
     * @return The magnitude of the vector
     */
    public double getMagnitude() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }
    /**
     * Gets the vector angle in radians.
     *
     * @return The angle of the vector, in radians
     */
    public double getAngle() {
        return Math.atan2(this.y, this.x);
    }

    /**
     * Returns the slope of the vector.
     * @return the rise-over-run (y over x) slope of the vector
     */
    public double getSlope() {
        return this.y / this.x;
    }

    /**
     * Sets a new angle and magnitude for the vector.
     * @param angleRadians The new angle, in radians
     * @param mag The new magnitude
     */
    public void setAngleAndMagnitude(double angleRadians, double mag) {
        this.x = Math.cos(angleRadians) * mag;
        this.y = Math.sin(angleRadians) * mag;
    }

    /**
     * Sets a new magnitude for the vector while maintaining its current angle.
     * @param mag The new magnitude
     */
    public void setMagnitude(double mag) {
        this.setAngleAndMagnitude(getAngle(), mag);
    }

    /**
     * Sets a new angle for the vector while maintaining its current magnitude.
     * @param angle The new angle, in radians
     */
    public void setAngle(double angle) {
        this.setAngleAndMagnitude(angle, getMagnitude());
    }

    public double distanceFrom(Vector2d vect) {
        return Math.sqrt(
                Math.pow(this.x - vect.x, 2) +
                Math.pow(this.y - vect.y, 2)
        );
    }

    public Vector2d rotated(double radians) {
        return this.withAngle(this.getAngle() + radians);
    }

    public Vector2d withAngle(double angle) {
        Vector2d ret = new Vector2d(this.x, this.y);
        ret.setAngle(angle);

        return ret;
    }

    public Vector2d withMagnitude(double mag) {
        Vector2d ret = new Vector2d(this.x, this.y);
        ret.setMagnitude(mag);

        return ret;
    }

    public Vector2d perpendicular() {
        return new Vector2d(this.x, -this.y);
    }
}
