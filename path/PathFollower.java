package org.firstinspires.ftc.teamcode.livewire.path;

import org.firstinspires.ftc.teamcode.livewire.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.livewire.path.points.HeadingStrategy;
import org.firstinspires.ftc.teamcode.livewire.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Line2d;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.DistanceParameterizedVelocityGenerator;
import org.firstinspires.ftc.teamcode.livewire.util.pid.constants.ProfiledControllerConstants;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.PIDFController;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.VelocityController;

public class PathFollower {
    private MecanumDrivetrain drivetrain;
    private Path path;

    private DistanceParameterizedVelocityGenerator distanceProfiledVelocityGenerator;
    /**
     * There are two velocity controllers for forward and lateral. Since mecanum drivetrains have greater friction when pushed side to side,
     * this combats that by using two tuners for forward and lateral.
     */
    private VelocityController forwardVelocityController;
    private VelocityController lateralVelocityController;
    // Controls the translational (side-to-side error) vector of the follower
    private PIDFController translationalPIDFController;
    private PIDFController rotationalPIDFController;

    // lastWaypoint is the waypoint the robot is coming from. nextWaypoint is the waypoint the robot is approaching.
    // Both are needed for accel/decel
    private Waypoint lastWaypoint;
    private Waypoint nextWaypoint;
    private Pose2d currentPose;
    // The current velocity of the robot, but robot centric. This is necessary to use two forward/lateral velocity controllers.
    private Pose2d currentVelocityRobotCentric;
    private double currentVelocityAlongPathLine = 0;
    // This is the target velocity the robot must follow, but converted to robot centric.
    private Vector2d targetVelocityRobotCentric;
    private Line2d pathLine;  // the target path represented as a Line2d
    private Vector2d pathVector;

    // the current waypoint the robot is leaving from
    private int currentWaypointIndex = 0;

    private double distanceFromNextWaypoint;
    private double targetVelocity;
    private double targetRotation;
    private double perpendicularError;

    private boolean isFinished = true;

    /**
     * Class that generates vectors to follow to maintain a path.
     * @param drivetrain drivetrain
     */
    public PathFollower(MecanumDrivetrain drivetrain) {
        this.forwardVelocityController = new VelocityController(MecanumDrivetrain.FORWARD_VELOCITY_CONSTANTS);
        this.lateralVelocityController = new VelocityController(MecanumDrivetrain.LATERAL_VELOCITY_CONSTANTS);

        this.distanceProfiledVelocityGenerator = new DistanceParameterizedVelocityGenerator();
        this.translationalPIDFController = new PIDFController(MecanumDrivetrain.TRANSLATIONAL_PID_CONSTANTS);
        this.rotationalPIDFController = new PIDFController(MecanumDrivetrain.ROTATIONAL_PID_CONSTANTS);
    }

    /**
     * Gets a path ready to run by resetting the necessary parameters. This should be called only once at the start, and should be followed by
     * subsequent update() calls.
     * @param path
     */
    public void startPath(Path path) {
        this.path = path;
        this.isFinished = false;

        this.setWaypoint(0);
    }

    public void update() {
        this.currentPose = this.drivetrain.getPose();
        this.currentVelocityRobotCentric = this.drivetrain.getRobotRelativeVelocity();

        // uses the pythag theorem to calculate the velocity only in the path's direction
        this.currentVelocityAlongPathLine = Math.sqrt((this.drivetrain.getFieldRelativeVelocity().vector.getMagnitude()) +
                this.pathLine.distanceToPoint(this.drivetrain.getFieldRelativeVelocity().vector));

        // checks if the drivetrain is going in the direction of the path by checking its angle compared to the path angle.
        // if it's reversed, then the current velocity along the path is negated
        if ((this.currentPose.heading - this.pathVector.getAngle()) > Math.PI) {
            this.currentVelocityAlongPathLine = -this.currentVelocityAlongPathLine;
        }

        // Calculating target heading
        if (this.nextWaypoint.headingStrategy == HeadingStrategy.INSTANT) {
            this.targetRotation = this.nextWaypoint.point.heading;
        }
        else if (this.nextWaypoint.headingStrategy == HeadingStrategy.INTERPOLATION) {
            double percentAlongPath = (this.pathVector.getMagnitude() - this.distanceFromNextWaypoint) / this.pathVector.getMagnitude();
            this.targetRotation = this.lastWaypoint.point.heading + (this.nextWaypoint.point.heading - this.lastWaypoint.point.heading) * percentAlongPath;
        }
        else if (this.nextWaypoint.headingStrategy == HeadingStrategy.POINT_TRACKING) {
            this.targetRotation = Math.atan2(this.nextWaypoint.trackedPoint.getY() - this.currentPose.getY(), this.nextWaypoint.trackedPoint.getX() - this.currentPose.getX());
        }


        // distance from the target waypoint
        this.distanceFromNextWaypoint = this.currentPose.distanceFrom(this.nextWaypoint.point);

        // generates a target velocity based on the distance from the target waypoint
        this.targetVelocity = this.distanceProfiledVelocityGenerator.update(this.distanceFromNextWaypoint, this.currentVelocityAlongPathLine);
        this.targetVelocityRobotCentric = this.pathVector.withMagnitude(targetVelocity).rotated(-this.currentPose.heading);

        // the perpendicular distance (error) from the path line that the robot must correct for.
        this.perpendicularError = this.pathLine.distanceToPoint(this.currentPose.vector);

        Vector2d calculatedDriveVector = new Vector2d(
                this.forwardVelocityController.update(this.currentVelocityRobotCentric.getX(), this.targetVelocityRobotCentric.x),
                this.lateralVelocityController.update(this.currentVelocityRobotCentric.getY(), this.targetVelocityRobotCentric.y)
        ).rotated(-this.currentPose.heading);  // converting it from robot centric back to field centric

        Vector2d calculatedTranslationalVector = this.pathVector.perpendicular().withMagnitude(
                this.translationalPIDFController.update(this.perpendicularError));


        this.drivetrain.setToFieldRelativePercent(
                calculatedDriveVector.plus(calculatedTranslationalVector),
                this.rotationalPIDFController.update(this.targetRotation - this.currentPose.heading)
        );


        // check to see if it has passed its nextWaypoint
        if (this.distanceFromNextWaypoint < this.nextWaypoint.tolerance) {
            this.currentWaypointIndex ++;

            if (currentWaypointIndex == (this.path.waypoints.size()-1)) {
                this.isFinished = true;
            }
            else {
                this.setWaypoint(this.currentWaypointIndex);
            }
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

        this.distanceProfiledVelocityGenerator.reset(this.lastWaypoint.targetVelocity, this.nextWaypoint.targetVelocity, this.lastWaypoint.distanceFrom(this.nextWaypoint));
        this.distanceProfiledVelocityGenerator.setConstants(
                new ProfiledControllerConstants(
                    this.lastWaypoint.maxAccel,
                    this.nextWaypoint.maxDecel,
                    this.lastWaypoint.maxVelocity
                )
        );

        this.pathVector = this.nextWaypoint.point.vector.minus(this.lastWaypoint.point.vector);
        this.pathLine = new Line2d(this.lastWaypoint.point.vector, this.nextWaypoint.point.vector);
    }

    public boolean isFinished() {
        return isFinished;
    }
}
