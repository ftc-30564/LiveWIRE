package org.firstinspires.ftc.teamcode.path.points;

import org.firstinspires.ftc.teamcode.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;

public class Waypoint {
    /**
     * The desired 2d pose on the field, including x, y, and heading
     */
    public Pose2d point;
    /**
     * The max acceleration leaving the point, in inches per second squared
     */
    public double maxAccel;
    /**
     * The max deceleration approaching the point, in inches per second squared
     */
    public double maxDecel;
    /**
     * The max velocity the robot may travel from now on past this point.
     */
    public double maxVelocity;
    /**
     * The target velocity the robot should aim at when approaching the point, in inches per second
     */
    public double targetVelocity = 0;

    /**
     * The tolerance the robot should be within to consider itself as having reached the waypoint, in inches.
     */
    public double tolerance = 0.5;

    /**
     * The heading "strategy" when approaching a waypoint
     */
    public HeadingStrategy headingStrategy = HeadingStrategy.INTERPOLATION;

    public Pose2d trackedPoint = null;

    public Waypoint(Pose2d point) {
        this(point, MecanumDrivetrain.DEFAULT_ACCEL, MecanumDrivetrain.DEFAULT_DECEL, MecanumDrivetrain.DEFAULT_VELOCITY);
    }

    /**
     * Creates a new waypoint with the given point, max acceleration, and max deceleration.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel) {
        this(point, maxAccel, maxDecel, MecanumDrivetrain.DEFAULT_VELOCITY);
    }

    /**
     * Creates a new waypoint with the given point, max acceleration, max deceleration, velocity, and slow down ratio.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     * @param velocity The max velocity the robot should target during its path
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel, double velocity) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVelocity = velocity;
    }

    /**
     * Creates a new waypoint with the given point, max acceleration, max deceleration, velocity, and slow down ratio.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     * @param targetVelocity The target velocity the robot should finish with when crossing the point, in inches per second
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel, double velocity, double targetVelocity) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVelocity = velocity;
        this.targetVelocity = targetVelocity;
    }

    /**
     * Creates a new waypoint with the given point, max acceleration, max deceleration, velocity, and slow down ratio.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     * @param targetVelocity The target velocity the robot should finish with when crossing the point, in inches per second
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel, double velocity, double targetVelocity, double tolerance) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.maxVelocity = velocity;
        this.targetVelocity = targetVelocity;
        this.tolerance = tolerance;
    }

    public double distanceFrom(Waypoint waypoint) {
        return this.point.distanceFrom(waypoint.point);
    }
}
