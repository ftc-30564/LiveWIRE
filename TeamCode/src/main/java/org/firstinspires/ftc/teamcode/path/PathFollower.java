package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.Line2d;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.pid.controller.DistanceProfiledVelocityController;
import org.firstinspires.ftc.teamcode.util.pid.constants.ProfiledControllerConstants;

public class PathFollower {
    private Drivetrain drivetrain;
    private Path path;

    private DistanceProfiledVelocityController profiledPIDController;
    // lastWaypoint is the waypoint the robot is coming from. nextWaypoint is the waypoint the robot is approaching.
    // Both are needed for accel/decel
    private Waypoint lastWaypoint;
    private Waypoint nextWaypoint;
    private Pose2d currentPose;
    private Pose2d currentVelocity;

    private double targetVelocity;

    public PathFollower(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // gets a path ready to run
    public void queuePath(Path path) {
        this.path = path;

        this.lastWaypoint = path.waypoints.get(0);
        this.nextWaypoint = path.waypoints.get(1);

        this.profiledPIDController = new DistanceProfiledVelocityController(
                new ProfiledControllerConstants(
                        this.lastWaypoint.maxAccel,
                        this.nextWaypoint.maxDecel,
                        this.path.maxVelocity),
                this.lastWaypoint.velocity,
                this.nextWaypoint.velocity);

        this.profiledPIDController.reset(this.lastWaypoint.velocity, this.nextWaypoint.velocity);
    }

    public void followPath() {
        this.currentPose = this.drivetrain.getPose();
        this.currentVelocity = this.drivetrain.getVelocity();

        Line2d targetLine = new Line2d(this.lastWaypoint.point.vector(DistanceUnit.INCH), this.nextWaypoint.point.vector(DistanceUnit.INCH)));

        // the distance from the target line that the robot must follow.
        double translationalError = targetLine.distanceToPoint(this.currentPose.vector(DistanceUnit.INCH));

        // check to see if it has passed its nextWaypoint
    }
}
