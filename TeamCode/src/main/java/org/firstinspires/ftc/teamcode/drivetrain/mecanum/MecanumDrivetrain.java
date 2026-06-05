package org.firstinspires.ftc.teamcode.drivetrain.mecanum;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.localizer.PoseEstimator;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;
import org.firstinspires.ftc.teamcode.util.pid.constants.PIDSVAConstants;

public class MecanumDrivetrain implements Drivetrain {
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    private final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    private final MecanumConstants mecanumConstants;
    private final PIDSVAConstants pidsvaConstants;

    private final PoseEstimator poseEstimator;

    /**
     * A holonomic drivetrain class for a robot with mecanum wheels. This class uses the standard mecanum wheel formulas to calculate motor powers.
     * @param hardwareMap hardware map instance
     * @param poseEstimator pose estimator instance for localization
     * @param mecanumConstants a class containing the constants for a mecanum drive
     */
    public MecanumDrivetrain(HardwareMap hardwareMap, PoseEstimator poseEstimator, MecanumConstants mecanumConstants, PIDSVAConstants pidsvaConstants) {
        this.poseEstimator = poseEstimator;

        this.frontLeftMotor = hardwareMap.get(DcMotorEx.class, mecanumConstants.frontLeftMotorName);
        this.frontRightMotor = hardwareMap.get(DcMotorEx.class, mecanumConstants.frontRightMotorName);
        this.backLeftMotor = hardwareMap.get(DcMotorEx.class, mecanumConstants.backLeftMotorName);
        this.backRightMotor = hardwareMap.get(DcMotorEx.class, mecanumConstants.backRightMotorName);

        this.frontLeftMotor.setDirection(mecanumConstants.frontLeftMotorDirection);
        this.frontRightMotor.setDirection(mecanumConstants.frontRightMotorDirection);
        this.backLeftMotor.setDirection(mecanumConstants.backLeftMotorDirection);
        this.backRightMotor.setDirection(mecanumConstants.backRightMotorDirection);

        this.mecanumConstants = mecanumConstants;
        this.pidsvaConstants = pidsvaConstants;
    }

    /**
     * Gets the current pose of the robot from the pose estimator.
     * @return the current pose of the robot
     */
    @Override
    public Pose2d getPose() {
        return this.poseEstimator.getPose();
    }

    /**
     * Sets the current pose of the robot through the pose estimator. This is useful for resetting the robot's position on the field or for setting an initial position.
     * @param newPose the new pose to set
     */
    @Override
    public void setPose(Pose2d newPose) {
        this.poseEstimator.setPose(newPose);
    }

    /**
     * Gets the current field-relative velocity of the robot from the pose estimator, using only odometry (no vision)
     * @return the current velocity of the robot
     */
    @Override
    public Pose2d getVelocity() {
        return this.poseEstimator.getVelocity();
    }

    /**
     * Sets the motor powers to achieve a desired field-relative velocity. The input vector and rotation are in percentage, so they will be rotated by the robot's current heading to convert them to robot-relative coordinates before calculating motor powers.
     * @param vector the desired field-relative velocity vector
     * @param rotation the desired rotational velocity
     */
    @Override
    public void setToFieldVector(Vector2d vector, double rotation) {
        // convert the field relative vector to robot relative
        vector.setAngle(
                vector.getAngle() - this.poseEstimator.getPose().heading.getRadians()
        );

        this.setToRobotVector(vector, rotation);
    }

    /**
     * Sets the motor powers to achieve a desired robot-relative velocity. The input vector and rotation are in percentage, so they will be used directly to calculate motor powers.
     * This automatically desaturates wheel speeds if one of the desired motor percentages is greater than 1.
     * Note: this does not directly call the motors; but rather it sets the percentages that will later be called under update().
     * @param vector the desired robot-relative velocity vector
     * @param rotation the desired rotational velocity
     */
    @Override
    public void setToRobotVector(Vector2d vector, double rotation) {
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

    /**
     * Updates the motor powers and the pose estimator. This should be called in a loop to continuously update the drivetrain.
     */
    @Override
    public void update() {
        this.frontLeftMotor.setPower(this.frontLeftPower);
        this.frontRightMotor.setPower(this.frontRightPower);
        this.backLeftMotor.setPower(this.backLeftPower);
        this.backRightMotor.setPower(this.backRightPower);

        this.poseEstimator.update();
    }

    @Override
    public PIDSVAConstants getVelocityConstants() {
        return this.pidsvaConstants;
    }
}
