package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.Line2d;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.util.pid.constants.PIDSVAConstants;
import org.firstinspires.ftc.teamcode.util.pid.controller.DistanceProfiledVelocityGenerator;
import org.firstinspires.ftc.teamcode.util.pid.constants.ProfiledControllerConstants;
import org.firstinspires.ftc.teamcode.util.pid.controller.VelocityController;

public class PathFollower {
    private Drivetrain drivetrain;
    private Path path;

    private DistanceProfiledVelocityGenerator distanceProfiledVelocityGenerator;
    private VelocityController velocityController;

    // lastWaypoint is the waypoint the robot is coming from. nextWaypoint is the waypoint the robot is approaching.
    // Both are needed for accel/decel
    private Waypoint lastWaypoint;
    private Waypoint nextWaypoint;
    private Pose2d currentPose;
    private Pose2d currentVelocity;
    private Line2d targetLine;

    private int currentWaypointIndex = 0;

    private double distanceFromNextWaypoint;
    private double targetVelocity;
    private double perpendicularError;

    public PathFollower(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.distanceProfiledVelocityGenerator = new DistanceProfiledVelocityGenerator();
        this.velocityController = new VelocityController(new PIDSVAConstants(0,0,0,0,0,0)); // TODO: figure out how to obtain constants
    }

    /**
     * Gets a path ready to run by resetting the necessary parameters. This should be called only once at the start, and should be followed by
     * subsequent update() calls.
     * @param path
     */
    public void startPath(Path path) {
        this.path = path;

        this.setWaypoints(0);
    }

    public void update() {
        this.currentPose = this.drivetrain.getPose();
        this.currentVelocity = this.drivetrain.getVelocity();

        this.distanceFromNextWaypoint = this.currentPose.distanceFrom(this.nextWaypoint.point).inches();

        // the perpendicular distance from the target line that the robot must correct for.
        this.perpendicularError = targetLine.distanceToPoint(this.currentPose.vector(DistanceUnit.INCH));
        this.targetVelocity = this.distanceProfiledVelocityGenerator.update(this.distanceFromNextWaypoint);
        // TODO: figure out targetHeading
        

        // check to see if it has passed its nextWaypoint
        if (this.distanceFromNextWaypoint < this.nextWaypoint.tolerance) {
            this.currentWaypointIndex ++;

            this.setWaypoints(this.currentWaypointIndex);
        }
    }

    /**
     * Sets the new target waypoints (last and next) based on a given index, and sets the appropriate parameters.
     * @param index the index to set. This index should refer to the last waypoint in the path.
     */
    private void setWaypoints(int index) {
        this.lastWaypoint = this.path.waypoints.get(index);
        this.nextWaypoint = this.path.waypoints.get(index+1);

        this.distanceProfiledVelocityGenerator.reset(this.lastWaypoint.targetVelocity, this.nextWaypoint.targetVelocity);
        this.distanceProfiledVelocityGenerator.setConstants(
                new ProfiledControllerConstants(
                    this.lastWaypoint.maxAccel,
                    this.nextWaypoint.maxDecel,
                    this.lastWaypoint.maxVelocity
                )
        );

        this.targetLine = new Line2d(this.lastWaypoint.point.vector(DistanceUnit.INCH), this.nextWaypoint.point.vector(DistanceUnit.INCH));
    }
}
