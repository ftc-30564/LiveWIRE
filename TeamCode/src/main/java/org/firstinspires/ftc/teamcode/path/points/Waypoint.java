package org.firstinspires.ftc.teamcode.path.points;

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
     * The target velocity the robot should aim at when approaching the point, in inches per second
     */
    public double velocity = 0;
    /**
     * The ratio of how much the robot should slow down when it encounters a sharp angle change in the path,
     * ex. a 90 degree turn. This is multiplied by the cosine of the angle change. A higher ratio means the robot
     * will slow down more agressively, while a lower ratio means the robot won't slow down as much.
     */
    public double slowDownRatio = 1;

    /**
     * Creates a new waypoint with the given point, max acceleration, and max deceleration.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
    }

    /**
     * Creates a new waypoint with the given point, max acceleration, max deceleration, and velocity.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     * @param velocity The target velocity the robot should aim at when approaching the point, in inches per second
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel, double velocity) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.velocity = velocity;
    }

    /**
     * Creates a new waypoint with the given point, max acceleration, max deceleration, velocity, and slow down ratio.
     * @param point The desired 2d pose on the field, including x, y, and heading
     * @param maxAccel The max acceleration leaving the point, in inches per second squared
     * @param maxDecel The max deceleration approaching the point, in inches per second squared
     * @param velocity The target velocity the robot should aim at when approaching the point, in inches per second
     * @param slowDownRatio The ratio of how much the robot should slow down when it encounters a sharp angle change in the path. Higher means it slows down more, lower means less.
     */
    public Waypoint(Pose2d point, double maxAccel, double maxDecel, double velocity, double slowDownRatio) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.velocity = velocity;
        this.slowDownRatio = slowDownRatio;
    }
}
