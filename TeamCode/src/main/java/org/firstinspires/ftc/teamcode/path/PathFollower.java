package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.Line2d;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.util.pid.constants.PIDFConstants;
import org.firstinspires.ftc.teamcode.util.pid.constants.PIDSVAConstants;
import org.firstinspires.ftc.teamcode.util.pid.controller.DistanceParameterizedVelocityGenerator;
import org.firstinspires.ftc.teamcode.util.pid.constants.ProfiledControllerConstants;
import org.firstinspires.ftc.teamcode.util.pid.controller.PIDFController;
import org.firstinspires.ftc.teamcode.util.pid.controller.VelocityController;

public class PathFollower {
    private Drivetrain drivetrain;
    private Path path;

    private DistanceParameterizedVelocityGenerator distanceProfiledVelocityGenerator;
    private VelocityController driveVelocityController;
    private PIDFController translationalPIDFController;

    // lastWaypoint is the waypoint the robot is coming from. nextWaypoint is the waypoint the robot is approaching.
    // Both are needed for accel/decel
    private Waypoint lastWaypoint;
    private Waypoint nextWaypoint;
    private Pose2d currentPose;
    private Pose2d currentVelocity;
    private Line2d pathLine;  // the target path represented as a Line2d
    private Vector2d pathVector;

    // the current waypoint the robot is leaving from
    private int currentWaypointIndex = 0;

    private double distanceFromNextWaypoint;
    private double targetVelocity;
    private double perpendicularError;

    public PathFollower(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.distanceProfiledVelocityGenerator = new DistanceParameterizedVelocityGenerator();
        this.driveVelocityController = new VelocityController(drivetrain.getVelocityConstants()); // TODO: figure out how to obtain constants
        this.translationalPIDFController = new PIDFController(new PIDFConstants(0,0,0,0));
    }

    /**
     * Gets a path ready to run by resetting the necessary parameters. This should be called only once at the start, and should be followed by
     * subsequent update() calls.
     * @param path
     */
    public void startPath(Path path) {
        this.path = path;

        this.setWaypoint(0);
    }

    public void update(double dt) {
        this.currentPose = this.drivetrain.getPose();
        this.currentVelocity = this.drivetrain.getVelocity();

        /* The velocity of the robot that is adjusted to purely in the drive vector direction.
         it calculates one of the sides of a right triangle with the known hypotenuse (velocity)
         and a known side (distance from velocity endpoint to the main line)
         */
        double currentDriveVectorVelocity = Math.sqrt(Math.pow(this.currentVelocity.vector(DistanceUnit.INCH).getMagnitude(), 2)
                - Math.pow(this.pathLine.distanceToPoint(this.currentVelocity.vector(DistanceUnit.INCH)), 2));

        // distance from the target waypoint
        this.distanceFromNextWaypoint = this.currentPose.distanceFrom(this.nextWaypoint.point).inches();

        // the perpendicular distance (error) from the path line that the robot must correct for.
        this.perpendicularError = this.pathLine.distanceToPoint(this.currentPose.vector(DistanceUnit.INCH));
        // generates a target velocity based on the distance from the target waypoint
        this.targetVelocity = this.distanceProfiledVelocityGenerator.update(this.distanceFromNextWaypoint);

        Vector2d calculatedDriveVector = this.pathVector.withMagnitude(
                this.driveVelocityController.update(currentDriveVectorVelocity, this.targetVelocity));

        Vector2d calculatedTranslationalVector = this.pathVector.perpendicular().withMagnitude(
                this.translationalPIDFController.update(this.perpendicularError, dt));


        // TODO: figure out rotation
        this.drivetrain.setToFieldVector(calculatedDriveVector.plus(calculatedTranslationalVector), 0);

        // check to see if it has passed its nextWaypoint
        if (this.distanceFromNextWaypoint < this.nextWaypoint.tolerance) {
            this.currentWaypointIndex ++;

            this.setWaypoint(this.currentWaypointIndex);
        }
    }

    /**
     * Sets the new target waypoints (last and next) based on a given index, and sets the appropriate parameters.
     * @param index the index to set. This index should refer to the waypoint the robot is leaving; so for example, if the
     * robot is just starting out its path, it should call setWaypoint(0), since waypoint 0 is the waypoint it is leaving from.
     */
    private void setWaypoint(int index) {
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

        this.pathVector = this.nextWaypoint.point.vector(DistanceUnit.INCH).minus(this.lastWaypoint.point.vector(DistanceUnit.INCH));
        this.pathLine = new Line2d(this.lastWaypoint.point.vector(DistanceUnit.INCH), this.nextWaypoint.point.vector(DistanceUnit.INCH));
    }
}
