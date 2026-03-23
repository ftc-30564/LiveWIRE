package org.firstinspires.ftc.teamcode.util.math;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.math.units.Distance;

public class Pose2d {
    public Distance x;
    public Distance y;
    public Rotation2d heading;

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param vector translational (x and y) element in the form of a vector, in inches.
     * @param theta rotation, in radians
     */
    public Pose2d(Vector2d vector, Rotation2d theta) {
        this(vector, theta, DistanceUnit.INCH);
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param vector translational (x and y) element in the form of a vector
     * @param theta rotation, in radians
     * @param unit the distance unit that the vector is in.
     */
    public Pose2d(Vector2d vector, Rotation2d theta, DistanceUnit unit) {
        this(Distance.fromUnit(vector.x, unit), Distance.fromUnit(vector.y, unit), theta);
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param x x translation, in inches
     * @param y y translation, in inches
     * @param theta rotation, in radians
     */
    public Pose2d(double x, double y, double theta) {
        this.x = Distance.fromInches(x);
        this.y = Distance.fromInches(y);
        this.heading = Rotation2d.fromRadians(theta);
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     * @param x x translation, as a Distance object
     * @param y y translation, as a Distance object
     * @param theta rotation as a Rotation2d
     */
    public Pose2d(Distance x, Distance y, Rotation2d theta) {
        this.x = x;
        this.y = y;
        this.heading = theta;
    }

    /**
     * Constructor for Pose2d. A class containing a 2d translational and a rotational element, often used to represent the robot's current state on the field.
     */
    public Pose2d() {
        this.x = new Distance();
        this.y = new Distance();
        this.heading = new Rotation2d();
    }

    /**
     * Returns the X of the pose
     * @return x of the pose, as a Distance
     */
    public Distance getX() {
        return this.x;
    }

    /**
     * Returns the Y of the pose
     * @return y of the pose, as a Distance object
     */
    public Distance getY() {
        return this.y;
    }

    /**
     * Returns the heading of the pose
     * @return heading as a Rotation2d
     */
    public Rotation2d getHeading() {
        return this.heading;
    }

    /**
     * Returns the translational element as a vector.
     * @param distanceUnit the unit to set the vector to
     * @return x and y as a vector
     */
    public Vector2d vector(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH)
            return new Vector2d(this.x.inches(), this.y.inches());
        return new Vector2d(this.x.meters(), this.y.meters());
    }

    /**
     * Adds two poses together.
     * @param pose the pose to add
     * @return a new pose with the X, Y and headings combined.
     */
    public Pose2d add(Pose2d pose) {
        return new Pose2d(
                this.x.plus(pose.x),
                this.y.plus(pose.y),
                this.heading.plus(pose.heading).wrapped());
    }

    /**
     * Adds a vector to the pose together.
     * @param vector the vector to add
     * @return a new pose with the X, Y and headings combined.
     */
    public Pose2d add(Vector2d vector, DistanceUnit unit) {
        return new Pose2d(
                this.x.plus(Distance.fromUnit(vector.x, unit)),
                this.y.plus(Distance.fromUnit(vector.y, unit)),
                this.heading);
    }

    /**
     * Subtracts two poses together.
     * @param pose the pose to subtract
     * @return a new pose with the X, Y and headings subtracted.
     */
    public Pose2d subtract(Pose2d pose) {
        return new Pose2d(
                this.x.minus(pose.x),
                this.y.minus(pose.y),
                this.heading.minus(pose.heading).wrapped());
    }

    /**
     * Subtracts a vector to the pose together.
     * @param vector the vector to subtract
     * @return a new pose with the X and Y subtracted.
     */
    public Pose2d subtract(Vector2d vector, DistanceUnit unit) {
        return new Pose2d(
                this.x.minus(Distance.fromUnit(vector.x, unit)),
                this.y.minus(Distance.fromUnit(vector.y, unit)),
                this.heading);
    }

    /**
     * Finds the distance between two poses using the Pythagorean Theorem
     * @param pose the pose to get the distance from
     */
    public Distance distanceFrom(Pose2d pose) {
        return Distance.fromInches(Math.sqrt(
                Math.pow(this.x.minus(pose.x).inches(), 2) +
                Math.pow(this.y.minus(pose.y).inches(), 2)));
    }

    /**
     * Finds the distance between the pose and a vector using the Pythagorean Theorem
     * @param vector the vector to get the distance from
     * @param unit the unit that the vector is in
     */
    public double distanceFrom(Vector2d vector, DistanceUnit unit) {
        return Math.sqrt(
                Math.pow(this.x.unit(unit) - vector.x, 2) +
                        Math.pow(this.y.unit(unit) - vector.y, 2));
    }

    public Pose2D pose2D(DistanceUnit distUnit, AngleUnit angleUnit) {
        return new Pose2D(
                distUnit,
                getX().unit(distUnit),
                getY().unit(distUnit),
                angleUnit,
                getHeading().getUnit(angleUnit));
    }
}
