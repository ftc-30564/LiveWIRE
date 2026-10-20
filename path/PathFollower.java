package org.firstinspires.ftc.teamcode.livewire.path;

import static org.firstinspires.ftc.teamcode.Constants.LOGGER;
import static org.firstinspires.ftc.teamcode.livewire.drivetrain.mecanum.MecanumDrivetrain.FORWARD_VELOCITY_CONSTANTS;
import static org.firstinspires.ftc.teamcode.livewire.drivetrain.mecanum.MecanumDrivetrain.LATERAL_VELOCITY_CONSTANTS;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.livewire.drivetrain.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.livewire.path.points.HeadingStrategy;
import org.firstinspires.ftc.teamcode.livewire.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.livewire.util.pid.constants.PIDSVAConstants;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.DistanceParameterizedVelocityGenerator;
import org.firstinspires.ftc.teamcode.livewire.util.pid.constants.ProfiledControllerConstants;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.PIDFController;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.VelocityController;

// This path generates output speeds based on a given path.
public class PathFollower {
    private MecanumDrivetrain drivetrain;
    private Path path;

    public static boolean DRAW = false;
    public static boolean LOG_FTC_DASH = false;
    public static boolean LOG_TELEMETRY = false;

    private DistanceParameterizedVelocityGenerator distanceProfiledVelocityGenerator;
    private VelocityController driveVelocityController;

    // The adjusted constants, blended from the forward/lateral constants depending on what angle the drivetrain is demanded.
    private PIDSVAConstants adjustedDriveVelocityConstants = new PIDSVAConstants();

    // Controls the translational (side-to-side error) vector of the follower
    private PIDFController translationalPIDFController;
    private PIDFController rotationalPIDFController;
    private PIDFController holdPointPIDFController;

    // lastWaypoint is the waypoint the robot is coming from. nextWaypoint is the waypoint the robot is approaching.
    // Both are needed for accel/decel
    private Waypoint lastWaypoint;
    private Waypoint nextWaypoint;

    private Pose2d globalPose;
    private Pose2d relativeToStartPose;

    private double currentVelocityAlongPathLine = 0;
    // This is the target velocity the robot must follow, but converted to robot centric.
    private Vector2d pathVector;
    private Vector2d pathUnitVector;

    // the current waypoint the robot is leaving from
    private int currentWaypointIndex = 0;

    private double totalPathDistance = 0;
    private double distanceFromNextWaypoint;
    private double distanceFromNextWaypointAlongPathLine;
    private double targetVelocity;
    private double targetRotation;
    private double perpendicularError;

    private Vector2d calculatedDriveVector;
    private Vector2d calculatedTranslationalVector;

    private boolean isFinished = true;

    /**
     * Class that generates vectors to follow to maintain a path.
     * @param drivetrain drivetrain
     */
    public PathFollower(MecanumDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.driveVelocityController = new VelocityController(FORWARD_VELOCITY_CONSTANTS);

        this.distanceProfiledVelocityGenerator = new DistanceParameterizedVelocityGenerator();

        this.translationalPIDFController = new PIDFController(MecanumDrivetrain.TRANSLATIONAL_PID_CONSTANTS);
        this.rotationalPIDFController = new PIDFController(MecanumDrivetrain.ROTATIONAL_PID_CONSTANTS);
        this.holdPointPIDFController = new PIDFController(MecanumDrivetrain.HOLD_POINT_CONSTANTS);
    }

    private double blendConstant(double kForward, double kLateral, double angleRadians) {
        double cos2 = Math.pow(Math.cos(angleRadians), 2);
        double sin2 = Math.pow(Math.sin(angleRadians), 2);
        return kForward * cos2 + kLateral * sin2;
    }

    private void updateAdjustedConstants(double robotRelativeDirectionRadians) {
        this.adjustedDriveVelocityConstants.kS = blendConstant(FORWARD_VELOCITY_CONSTANTS.kS, LATERAL_VELOCITY_CONSTANTS.kS, robotRelativeDirectionRadians);
        this.adjustedDriveVelocityConstants.kV = blendConstant(FORWARD_VELOCITY_CONSTANTS.kV, LATERAL_VELOCITY_CONSTANTS.kV, robotRelativeDirectionRadians);
        this.adjustedDriveVelocityConstants.kA = blendConstant(FORWARD_VELOCITY_CONSTANTS.kA, LATERAL_VELOCITY_CONSTANTS.kA, robotRelativeDirectionRadians);
        this.adjustedDriveVelocityConstants.kP = blendConstant(FORWARD_VELOCITY_CONSTANTS.kP, LATERAL_VELOCITY_CONSTANTS.kP, robotRelativeDirectionRadians);
        this.adjustedDriveVelocityConstants.kS = blendConstant(FORWARD_VELOCITY_CONSTANTS.kI, LATERAL_VELOCITY_CONSTANTS.kI, robotRelativeDirectionRadians);
        this.adjustedDriveVelocityConstants.kS = blendConstant(FORWARD_VELOCITY_CONSTANTS.kD, LATERAL_VELOCITY_CONSTANTS.kD, robotRelativeDirectionRadians);

    }

    /**
     * Gets a path ready to run by resetting the necessary parameters. This should be called only once at the start, and should be followed by
     * subsequent update() calls.
     * @param path
     */
    public void startPath(Path path) {
        this.path = path;
        this.isFinished = false;
        this.currentWaypointIndex = 0;

        this.setWaypoint(0);
    }

    /**
     * Sends the necessary power to the drivetrain to follow the queued path. NOTE: This does not update the drivetrain. You must call drivetrain.update() outside of this.
     */
    public void update() {
        this.globalPose = this.drivetrain.getPose();
        this.relativeToStartPose = this.globalPose.minus(this.lastWaypoint.point.vector);  // the pose of the robot, with the start pose of the path as the origin

        this.currentVelocityAlongPathLine = this.pathUnitVector.dot(this.drivetrain.getFieldRelativeVelocity().vector);
        this.distanceFromNextWaypoint = this.globalPose.distanceFrom(this.nextWaypoint.point);// * (isPastEnd ? -1 : 1);
        this.distanceFromNextWaypointAlongPathLine = this.totalPathDistance - this.pathUnitVector.dot(this.relativeToStartPose.vector);

        // Calculating target heading
        if (this.nextWaypoint.headingStrategy == HeadingStrategy.INSTANT) {
            this.targetRotation = this.nextWaypoint.point.heading;
        }
        else if (this.nextWaypoint.headingStrategy == HeadingStrategy.INTERPOLATION) {
            double percentAlongPath = (this.totalPathDistance - this.distanceFromNextWaypointAlongPathLine) / this.totalPathDistance;
            this.targetRotation = this.lastWaypoint.point.heading + ((this.nextWaypoint.point.heading - this.lastWaypoint.point.heading) * percentAlongPath);
        }
        else if (this.nextWaypoint.headingStrategy == HeadingStrategy.POINT_TRACKING) {
            this.targetRotation = Math.atan2(this.nextWaypoint.trackedPoint.getY() - this.globalPose.getY(), this.nextWaypoint.trackedPoint.getX() - this.globalPose.getX());
        }

        // generates a target velocity based on the distance from the target waypoint
        this.targetVelocity = this.distanceProfiledVelocityGenerator.update(Math.abs(this.distanceFromNextWaypointAlongPathLine), this.currentVelocityAlongPathLine);
        this.targetVelocity *= Math.signum(this.distanceFromNextWaypointAlongPathLine);

        // the perpendicular distance (error) from the path line that the robot must correct for.
        this.perpendicularError = -this.pathUnitVector.cross(this.relativeToStartPose.vector);

        // blend the forward/lateral constants based on the demanded velocity relative to the robot
        this.updateAdjustedConstants(this.pathUnitVector.getAngle() - this.drivetrain.getPose().heading);

        this.driveVelocityController.setConstants(this.adjustedDriveVelocityConstants);

        if (this.isFinished) {
            this.calculatedDriveVector = this.pathUnitVector.times(
                    this.holdPointPIDFController.update(
                            this.distanceFromNextWaypointAlongPathLine
                    )
            );
        }
        else {
            this.calculatedDriveVector = this.pathUnitVector.times(
                    this.driveVelocityController.update(
                            this.currentVelocityAlongPathLine,
                            this.targetVelocity
                    )
            );
        }

        this.calculatedTranslationalVector = this.pathUnitVector.perpendicular().times(
                this.translationalPIDFController.update(this.perpendicularError));

        double headingError = (this.globalPose.heading) - this.targetRotation;

        if (Math.abs(headingError) > Math.PI) {
            headingError -= (Math.PI * 2) * Math.signum(headingError);
        }

        this.drivetrain.setToFieldRelativePercent(
                calculatedDriveVector.plus(calculatedTranslationalVector),
                this.rotationalPIDFController.update(headingError
                )
        );

        // check to see if it has passed its nextWaypoint
        if ((this.distanceFromNextWaypoint < this.nextWaypoint.tolerance && (!isFinished))) {
            this.currentWaypointIndex ++;

            if (currentWaypointIndex == (this.path.waypoints.size()-1)) {
                this.isFinished = true;
            }
            else {
                this.setWaypoint(this.currentWaypointIndex);
            }
        }

        LOGGER.addData("PathFollower/Vel along path line", this.currentVelocityAlongPathLine, LOG_TELEMETRY, LOG_FTC_DASH);
        LOGGER.addData("PathFollower/Distance from next waypoint", this.distanceFromNextWaypoint, LOG_TELEMETRY, LOG_FTC_DASH);
        LOGGER.addData("PathFollower/Distance front next waypoint along path", this.distanceFromNextWaypointAlongPathLine, LOG_TELEMETRY, LOG_FTC_DASH);
        LOGGER.addData("PathFollower/Target velocity", this.targetVelocity, LOG_TELEMETRY, LOG_FTC_DASH);
        LOGGER.addData("PathFollower/Perp error", this.perpendicularError, LOG_TELEMETRY, LOG_FTC_DASH);
        LOGGER.addData("PathFollower/Target heading", this.targetRotation, LOG_TELEMETRY, LOG_FTC_DASH);

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
        this.pathUnitVector = this.pathVector.normalized();

        this.totalPathDistance = this.pathVector.getMagnitude();
    }

    public void refreshConstants() {
        this.translationalPIDFController.setConstants(MecanumDrivetrain.TRANSLATIONAL_PID_CONSTANTS);
        this.rotationalPIDFController.setConstants(MecanumDrivetrain.ROTATIONAL_PID_CONSTANTS);
    }

    public double getTargetVelocity() {
        return this.targetVelocity;
    }

    public boolean isFinished() {
        return isFinished;
    }
}
