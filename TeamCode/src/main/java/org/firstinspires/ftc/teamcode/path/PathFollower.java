package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.Line2d;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.util.pid.DistanceProfiledPIDController;
import org.firstinspires.ftc.teamcode.util.pid.ProfiledPIDConstants;

public class PathFollower {
    private Drivetrain drivetrain;
    private Path path;

    private DistanceProfiledPIDController profiledPIDController;
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

        this.profiledPIDController = new DistanceProfiledPIDController(new ProfiledPIDConstants(0,0,0,0,0,0,0,0,0), 0, 50);
        this.lastWaypoint = path.waypoints.get(0);
        this.nextWaypoint = path.waypoints.get(1);
        this.targetVelocity = path.maxVelocity;

        this.profiledPIDController.reset(this.lastWaypoint.velocity, this.nextWaypoint.velocity);
    }

    public void followPath() {
        this.currentPose = this.drivetrain.getPose();
        this.currentVelocity = this.drivetrain.getVelocity();

        Vector2d closestPointToPath = closestPointToLine(this.currentPose.vector(DistanceUnit.INCH),
                new Line2d(this.lastWaypoint.point.vector(DistanceUnit.INCH), this.nextWaypoint.point.vector(DistanceUnit.INCH)));

        // looks at the distance from the robot's current position and the closest point on the path
        double translationalError = this.currentPose.vector(DistanceUnit.INCH).minus(closestPointToPath).getMagnitude();

        // check to see if it has passed its nextWaypoint
    }

    /**
     * Calculates the point on a line which is the closest to a reference point. Could probably use a better name but whatever. Does this by calculating
     * the point that intercepts two lines: the given line, and the perpendicular line that intersects the reference point.
     * @param point the point outside the line
     * @return a point inside a line that is closest to the input point
     */
    public Vector2d closestPointToLine(Vector2d point, Line2d line) {
        Line2d perpendicularLine = line.perpendicularLine(point);
        return line.pointOfIntersection(perpendicularLine);
    }
}
