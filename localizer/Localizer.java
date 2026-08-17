package org.firstinspires.ftc.teamcode.livewire.localizer;

import org.firstinspires.ftc.teamcode.livewire.util.math.units.Pose2d;

public interface Localizer {
    /**
     * Gets the current global pose of the robot in inches.
     * @return The current global pose of the robot, in inches.
     */
    Pose2d getPose();

    /**
     * Sets the current global pose of the robot.
     * @return The current global pose of the robot.
     */
    void setPose(Pose2d pose);

    /**
     * Returns the velocity of the robot.
     * @return field-relative velocity
     */
    Pose2d getVelocity();

    /**
     * Status of the localizer
     * @return status
     */
    default LocalizerStatus getStatus() {
        return LocalizerStatus.READY;
    };

    /**
     * Updates the localizer, should be called once per loop
     */
    default void update() {};
}
