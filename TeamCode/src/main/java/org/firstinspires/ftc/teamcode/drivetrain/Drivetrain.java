package org.firstinspires.ftc.teamcode.drivetrain;

import org.firstinspires.ftc.teamcode.localizer.Localizer;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;

public interface Drivetrain {
    /**
     * Gets the current pose of the robot from the pose estimator.
     * @return the current pose of the robot
     */
    Pose2d getPose();

    /**
     * Sets the current pose of the robot through the pose estimator. This is useful for resetting the robot's position on the field or for setting an initial position.
     * @param newPose the new pose to set
     */
    void setPose(Pose2d newPose);

    /**
     * Gets the current field-relative velocity of the robot from the pose estimator, using only dead wheels (no vision)
     * @return the current velocity of the robot
     */
    Pose2d getVelocity();

    /**
     * Sets the motor powers. Should be called in a loop to continuously update the drivetrain.
     */
    void update();

    /**
     * Sets the motor powers to achieve a desired field-relative velocity. The input vector and rotation are in percentage, so they will be rotated by the robot's current heading to convert them to robot-relative coordinates before calculating motor powers.
     * @param vector the desired field-relative velocity vector
     * @param rotation the desired rotational velocity
     */
    void setToFieldVector(Vector2d vector, double rotation);

    /**
     * Sets the motor powers to achieve a desired robot-relative velocity. The input vector and rotation are in percentage, so they will be used directly to calculate motor powers.
     * @param vector the desired robot-relative velocity vector
     * @param rotation the desired rotational velocity
     */
    void setToRobotVector(Vector2d vector, double rotation);
}
