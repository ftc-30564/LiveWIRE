package org.firstinspires.ftc.teamcode.livewire.util.math.units;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Pose2d {
    public Vector2d vector;
    public double heading;
// TODO: maybe don't do distance?
    public enum CoordinateSystem {
        STANDARD(0, 0, 0),
        PEDRO_PATHING(-72, -72, 0);

        double translationX;
        double translationY;
        double rotation;

        /**
         * Coordinate system enum for converting across coordinates
         * @param translationX translation x, inches
         * @param translationY translation y, inches
         * @param rotation rotation, inches. NOTE: rotation doesn't do anything at the moment
         */
        CoordinateSystem(double translationX, double translationY, double rotation) {
            this.translationX = translationX;
            this.translationY = translationY;
            this.rotation = rotation;
        }
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param vector translational (x and y) element in the form of a vector, in inches.
     * @param theta rotation, in radians
     */
    public Pose2d(Vector2d vector, Rotation2d theta) {
        this(vector, theta.getRadians());
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param vector translational (x and y) element in the form of a vector
     * @param theta rotation, in radians
     */
    public Pose2d(Vector2d vector, double theta) {
        this.vector = vector;
        this.heading = theta;
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param x x translation, in inches
     * @param y y translation, in inches
     */
    public Pose2d(double x, double y) {
        this.vector = new Vector2d(x, y);
        this.heading = 0;
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param x x translation, in inches
     * @param y y translation, in inches
     * @param theta rotation, in radians
     */
    public Pose2d(double x, double y, double theta) {
        this.vector = new Vector2d(x, y);
        this.heading = theta;
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param x x translation, as a Distance object
     * @param y y translation, as a Distance object
     * @param theta rotation as a Rotation2d
     */
    public Pose2d(Distance x, Distance y, Rotation2d theta) {
        this.vector = new Vector2d(x.inches(), y.inches());
        this.heading = theta.getRadians();
    }

    public Pose2d(Pose2D pose2D) {
        this(
                pose2D.getX(DistanceUnit.INCH),
                pose2D.getY(DistanceUnit.INCH),
                pose2D.getHeading(AngleUnit.RADIANS)
        );
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     */
    public Pose2d() {
        this.vector = new Vector2d();
        this.heading = 0;
    }

    /**
     * Returns the X of the pose
     * @return x of the pose, in inches
     */
    public double getX() {
        return this.vector.x;
    }

    /**
     * Returns the Y of the pose
     * @return y of the pose, in inches
     */
    public double getY() {
        return this.vector.y;
    }

    /**
     * Returns the heading of the pose
     * @return heading as radians
     */
    public double getHeading() {
        return this.heading;
    }

    /**
     * Returns the translational element as a vector.
     * @return x and y as a vector
     */
    public Vector2d vector() {
        return this.vector;
    }

    /**
     * Sets every component to zero in the pose object.
     */
    public void zero() {
        this.vector.x = 0;
        this.vector.y = 0;
        this.heading = 0;
    }

    /**
     * Adds two poses together.
     * @param pose the pose to add
     * @return a new pose with the X, Y and headings combined.
     */
    public Pose2d plus(Pose2d pose) {
        return new Pose2d(
                this.vector.plus(pose.vector),
                this.heading + pose.heading);
    }

    public Pose2d plusInPlace(Pose2d pose2d) {
        this.vector.x += pose2d.vector.x;
        this.vector.y += pose2d.vector.y;
        this.heading += pose2d.heading;

        return this;
    }
    /**
     * Adds a vector to the pose together.
     * @param vector the vector to add
     * @return a new pose with the X, Y and headings combined.
     */
    public Pose2d plus(Vector2d vector) {
        return new Pose2d(
                this.vector.plus(vector),
                this.heading);
    }

    /**
     * Subtracts two poses together.
     * @param pose the pose to subtract
     * @return a new pose with the X, Y and headings subtracted.
     */
    public Pose2d minus(Pose2d pose) {
        return new Pose2d(
                this.vector.minus(pose.vector),
                this.heading - pose.heading);
    }

    /**
     * Subtracts a vector to the pose together.
     * @param vector the vector to subtract
     * @return a new pose with the X and Y subtracted.
     */
    public Pose2d minus(Vector2d vector) {
        return new Pose2d(
                this.vector.minus(vector),
                this.heading);
    }

    public Pose2d minusInPlace(Pose2d pose2d) {
        this.vector.x -= pose2d.vector.x;
        this.vector.y -= pose2d.vector.y;
        this.heading -= pose2d.heading;

        return this;
    }

    /**
     * Divides the current pose by another
     * @param pose the pose to divide (the denominator)
     * @return a new pose with the X, Y and headings divided
     */
    public Pose2d divided(Pose2d pose) {
        return new Pose2d(
                this.vector.divided(pose.vector),
                this.heading / pose.heading);
    }

    /**
     * Divides the current pose by another
     * @param num the number to divide (the denominator)
     * @return a new pose with the X, Y and headings divided by the number provided
     */
    public Pose2d divided(double num) {
        return new Pose2d(
                this.vector.divided(num),
                this.heading / num);
    }

    /**
     * Multiplies two poses together
     * @param pose the pose to multiply
     * @return the product of the two poses
     */
    public Pose2d times(Pose2d pose) {
        return new Pose2d(
                this.vector.x * pose.vector.x,
                this.vector.y * pose.vector.y,
                this.heading * pose.heading
        );
    }

    public Pose2d timesInPlace(Pose2d pose2d) {
        this.vector.x *= pose2d.vector.x;
        this.vector.y *= pose2d.vector.y;
        this.heading *= pose2d.heading;

        return this;
    }

    /**
     * Returns a new pose with all its components absolute-valued
     * @return new pose with absolute-valued components
     */
    public Pose2d abs() {
        return new Pose2d(
                Math.abs(this.vector.x),
                Math.abs(this.vector.y),
                Math.abs(this.heading)
                );
    }

    /**
     * Returns the existing pose with all its components absolute-valued
     * Note: this does NOT create a new pose, but rather saves all the components
     * as absolute value and then returns itself.
     * @return itself with absolute-valued components
     */
    public Pose2d absInPlace() {
        this.vector.x = Math.abs(this.vector.x);
        this.vector.y = Math.abs(this.vector.y);
        this.heading = Math.abs(this.heading);

        return this;
    }

    /**
     * Finds the distance between two poses using the Pythagorean Theorem
     * @param pose the pose to get the distance from
     */
    public double distanceFrom(Pose2d pose) {
        return this.vector.distanceFrom(pose.vector);
    }

    /**
     * Finds the distance between the pose and a vector using the Pythagorean Theorem
     * @param vector the vector to get the distance from
     */
    public double distanceFrom(Vector2d vector) {
        return this.vector.distanceFrom(vector);
    }

    public Pose2d rotatedAroundPoint(Vector2d pivot, double radians) {
        Pose2d ret = new Pose2d(this.vector.minus(pivot), this.heading + radians);
        ret.vector.rotateInPlace(radians);
        ret.vector.plusInPlace(pivot);

        return ret;
    }


    /**
     * Mirrors the pose across a horizontal line.
     * @param y the y value of the line
     * @return
     */
    public Pose2d mirroredAcrossY(double y) {
        return new Pose2d(this.vector.x, y + (y - this.vector.y), heading);
    }

    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        return String.format("Pose2d: %.2f, %.2f, %.2f", this.vector.x, this.vector.y, this.heading);
    }

    public Pose2d toFTCStandard() {
        return new Pose2d((144-(72-this.getY()))-144, (72-(this.getX()+72))+72, this.heading - (Math.PI / 2));
    }

    public Pose2D pose2D() {
        return new Pose2D(
                DistanceUnit.INCH,
                this.vector.x,
                this.vector.y,
                AngleUnit.RADIANS,
                this.heading);
    }
}
