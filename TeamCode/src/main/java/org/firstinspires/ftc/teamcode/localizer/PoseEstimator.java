package org.firstinspires.ftc.teamcode.localizer;

import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;

public class PoseEstimator {
    private final Localizer localizer;

    private Pose2d estimatedPose;
    private Pose2d lastLocalizerPose;

    /**
     * Creates a new PoseEstimator with the specified localizer. The localizer will be used to update the estimated pose of the robot based on the localizer's readings.
     * Additionally, the class also takes in vision measurements, to fuse dead wheel odometry with vision for a more accurate pose estimation by using standard deviations. 
     * @param localizer the localizer to use for updating the estimated pose
     */
    public PoseEstimator(Localizer localizer) {
        this.localizer = localizer;
    }

    /**
     * Gets the current estimated pose of the robot from the pose estimator.
     * @return
     */
    public Pose2d getPose() {
        return estimatedPose;
    }

    /**
     * Sets the current estimated pose of the robot. This is useful for resetting the robot's position on the field or for setting an initial position.
     * @param pose
     */
    public void setPose(Pose2d pose) {
        this.estimatedPose = pose;
    }

    /**
     * Gets the current field-relative velocity of the robot from the pose estimator, using only odometry (no vision)
     * @return
     */
    public Pose2d getFieldCentricVelocity() {
        return localizer.getVelocity();
    }

    /**
     * Updates the estimated pose of the robot based on the localizer's readings. This should be called in a loop to continuously update the pose estimation. 
     */
    public void update() {
        Pose2d localizerPose = localizer.getPose();

        if (lastLocalizerPose == null) {
            lastLocalizerPose = localizerPose;
        }

        estimatedPose = estimatedPose.plus(
                localizerPose.minus(lastLocalizerPose)
        );
    }
}
