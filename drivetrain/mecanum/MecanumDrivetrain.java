package org.firstinspires.ftc.teamcode.livewire.drivetrain.mecanum;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.livewire.localizer.PoseEstimator;
import org.firstinspires.ftc.teamcode.livewire.util.math.MathUtil;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.livewire.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.livewire.util.pid.constants.PIDFConstants;
import org.firstinspires.ftc.teamcode.livewire.util.pid.constants.PIDSVAConstants;
import org.firstinspires.ftc.teamcode.livewire.util.pid.controller.VelocityController;

@Config
public class MecanumDrivetrain {
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    // TODO: change motor names
    public static final String FRONT_LEFT_MOTOR_NAME = "fl";
    public static final String FRONT_RIGHT_MOTOR_NAME = "fr";
    public static final String BACK_LEFT_MOTOR_NAME = "bl";
    public static final String BACK_RIGHT_MOTOR_NAME = "br";

    // TODO: change motor directions
    public static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // TODO: tune these constants
    public static PIDSVAConstants FORWARD_VELOCITY_CONSTANTS
            = new PIDSVAConstants(
            0.09,        // kS
            0.012,        // kV
            0.004,        // kA
            0.0025,        // kP
            0,        // kI
            0         // kD
    );

    // TODO: tune these constants
    public static PIDSVAConstants LATERAL_VELOCITY_CONSTANTS
            = new PIDSVAConstants(
            0.19,        // kS
            0.016,        // kV
            0.004,        // kA
            0.005,        // kP
            0,        // kI
            0         // kD
    );

    // TODO: tune these constants
    public static PIDFConstants TRANSLATIONAL_PID_CONSTANTS
            = new PIDFConstants(
        0.27,    // kP
            0,    // kI
            0.002,    // kD
            0     // kF
    );

    // TODO: tune these constants
    public static PIDFConstants ROTATIONAL_PID_CONSTANTS
            = new PIDFConstants(
            1,    // kP
            0,    // kI
            0.008,    // kD
            0     // kF
    );

    public static PIDFConstants HOLD_POINT_CONSTANTS = TRANSLATIONAL_PID_CONSTANTS;

    // The amount to cache the motors by. For example, if the motors are set to an amount that is within
    // the tolerance of the last percentage (aka a minimal change in percent), the motor will not be commanded.
    // This helps preserve loop times when motors are set to a constant percent.
    public static double MOTOR_CACHE = 0.03;

    public static final double DEFAULT_ACCEL = 100;
    public static final double DEFAULT_DECEL = -100;
    public static final double DEFAULT_VELOCITY = 50;
    public static final double DEFAULT_SLOWDOWN_RATIO = 1;

    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;

    private double lastFrontLeftPower = 0;
    private double lastFrontRightPower = 0;
    private double lastBackLeftPower = 0;
    private double lastBackRightPower = 0;

    private final PoseEstimator poseEstimator;

    /**
     * A holonomic drivetrain class for a robot with mecanum wheels. This class uses the standard mecanum wheel formulas to calculate motor powers.
     * @param hardwareMap hardware map instance
     * @param poseEstimator pose estimator instance for localization
     */
    public MecanumDrivetrain(HardwareMap hardwareMap, PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, MecanumDrivetrain.FRONT_LEFT_MOTOR_NAME);
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, MecanumDrivetrain.FRONT_RIGHT_MOTOR_NAME);
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, MecanumDrivetrain.BACK_LEFT_MOTOR_NAME);
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, MecanumDrivetrain.BACK_RIGHT_MOTOR_NAME);

        this.frontLeftMotor.setDirection(MecanumDrivetrain.FRONT_LEFT_MOTOR_DIRECTION);
        this.frontRightMotor.setDirection(MecanumDrivetrain.FRONT_RIGHT_MOTOR_DIRECTION);
        this.backLeftMotor.setDirection(MecanumDrivetrain.BACK_LEFT_MOTOR_DIRECTION);
        this.backRightMotor.setDirection(MecanumDrivetrain.BACK_RIGHT_MOTOR_DIRECTION);

        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Gets the current pose of the robot from the pose estimator.
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return this.poseEstimator.getPose();
    }

    /**
     * Sets the current pose of the robot through the pose estimator. This is useful for resetting the robot's position on the field or for setting an initial position.
     * @param newPose the new pose to set
     */
    public void setPose(Pose2d newPose) {
        this.poseEstimator.setPose(newPose);
    }

    /**
     * Gets the current field-relative velocity of the robot from the pose estimator, using only odometry (no vision)
     * @return the current velocity of the robot
     */
    public Pose2d getFieldRelativeVelocity() {
        return this.poseEstimator.getFieldCentricVelocity();
    }

    /**
     * Returns the velocity relative to the current angle the robot is pointing at.
     * @return
     */
    public Pose2d getRobotRelativeVelocity() {
        return new Pose2d(
                this.getFieldRelativeVelocity().vector.rotated(-this.poseEstimator.getPose().heading),
                this.getFieldRelativeVelocity().heading);
    }

    /**
     * Sets the motor powers to a desired field-relative percent. The input vector and rotation are in percentage, so they will be rotated by the robot's current heading to convert them to robot-relative coordinates before calculating motor powers.
     * @param vector the desired field-relative vector, in percentage
     * @param rotation the desired rotational percentage
     */
    public void setToFieldRelativePercent(Vector2d vector, double rotation) {
        // convert the field relative vector to robot relative
        vector.setAngle(
                vector.getAngle() - this.poseEstimator.getPose().heading
        );

        this.setToRobotRelativePercent(vector, rotation);
    }

    /**
     * Sets the motor powers to achieve a desired robot-relative percentage. The input vector and rotation are in percentage, so they will be used directly to calculate motor powers.
     * This automatically desaturates wheel speeds if one of the desired motor percentages is greater than 1.
     * Note: this does not directly call the motors; but rather it sets the percentages that will later be called under update().
     * @param vector the desired robot-relative velocity vector
     * @param rotation the desired rotational velocity
     */
    public void setToRobotRelativePercent(Vector2d vector, double rotation) {
        vector.setAngle(vector.getAngle() + (Math.PI/2));

        this.frontLeftPower  = vector.y + vector.x + rotation;
        this.frontRightPower = vector.y - vector.x - rotation;
        this.backLeftPower   = vector.y - vector.x + rotation;
        this.backRightPower  = vector.y + vector.x - rotation;

        // desaturate wheel speeds
        double maxWheelPower = Math.max(Math.max(this.frontLeftPower, this.frontRightPower), Math.max(this.backLeftPower, this.backRightPower));

        if (maxWheelPower > 1) {
            this.frontLeftPower *= 1 / maxWheelPower;
            this.frontRightPower *= 1 / maxWheelPower;
            this.backLeftPower *= 1 / maxWheelPower;
            this.backRightPower *= 1 / maxWheelPower;
        }
    }

    public void update() {
        this.update(true);
    }

    /**
     * Updates the motor powers and the pose estimator. This should be called in a loop to continuously update the drivetrain.
     * This will only update the motors if the commanded power is outside the cache range of what the motor is already set to.
     */
    public void update(boolean runMotors) {
        this.poseEstimator.update();

        if (!runMotors)
            return;

        if (!MathUtil.withinDeadband(this.lastFrontLeftPower, this.frontLeftPower, MecanumDrivetrain.MOTOR_CACHE)) {
            this.frontLeftMotor.setPower(this.frontLeftPower);
            this.lastFrontLeftPower = this.frontLeftPower;
        }

        if (!MathUtil.withinDeadband(this.lastFrontRightPower, this.frontRightPower, MecanumDrivetrain.MOTOR_CACHE)) {
            this.frontRightMotor.setPower(this.frontRightPower);
            this.lastFrontRightPower = this.frontRightPower;
        }

        if (!MathUtil.withinDeadband(this.lastBackLeftPower, this.backLeftPower, MecanumDrivetrain.MOTOR_CACHE)) {
            this.backLeftMotor.setPower(this.backLeftPower);
            this.lastBackLeftPower = this.backLeftPower;
        }

        if (!MathUtil.withinDeadband(this.lastBackRightPower, this.backRightPower, MecanumDrivetrain.MOTOR_CACHE)) {
            this.backRightMotor.setPower(this.backRightPower);
            this.lastBackRightPower = this.backRightPower;
        }
    }
}
