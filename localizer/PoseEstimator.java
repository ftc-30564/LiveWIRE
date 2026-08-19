package org.firstinspires.ftc.teamcode.livewire.localizer;

import org.firstinspires.ftc.teamcode.livewire.util.math.units.Pose2d;

public class PoseEstimator {
    private final Localizer localizer;

    // the last localizer-estimated pose, used for finding the delta
    private Pose2d lastLocalizerPose;
    private Pose2d poseUncertainty;

    private Pose2d estimatedPose;

    private final double localizerTranslationUncertainty = 0.01;
    private final double localizerRotationUncertainty = 0.01;

    private Pose2d latestVisionPose;
    private double latestVisionXUncertainty;
    private double latestVisionYUncertainty;
    private double latestVisionHeadingUncertainty;

    private Pose2d calculateLocalizerUncertainty(Pose2d deltas) {
        return deltas.times(new Pose2d(localizerTranslationUncertainty, localizerTranslationUncertainty, localizerRotationUncertainty));
    }

    /**
     * Creates a new PoseEstimator with the specified localizer. The localizer will be used to update the estimated pose of the robot based on the localizer's readings.
     * Additionally, the class also takes in vision measurements, to fuse dead wheel odometry with vision for a more accurate pose estimation by using standard deviations. 
     * @param localizer the localizer to use for updating the estimated pose
     */
    public PoseEstimator(Localizer localizer) {
        this.localizer = localizer;
        this.poseUncertainty = new Pose2d();
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

    public void addVisionMeasurement(Pose2d visionPose, double xDeviation, double yDeviation, double headingDeviation) {
        this.latestVisionPose = visionPose;
        this.latestVisionXUncertainty = xDeviation;
        this.latestVisionYUncertainty = yDeviation;
        this.latestVisionHeadingUncertainty = headingDeviation;
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
        if (this.lastLocalizerPose == null) {
            lastLocalizerPose = localizer.getPose();
        }

        // the "prediction" step, aka what the pinpoint says changed

        Pose2d predictedPose = this.estimatedPose.plus(localizer.getPose().minus(this.lastLocalizerPose));
        this.poseUncertainty = this.poseUncertainty.plus(calculateLocalizerUncertainty(localizer.getPose().minus(this.lastLocalizerPose)));


        if (this.latestVisionPose != null) {
            Pose2d innovation = this.latestVisionPose.minus(predictedPose);

            Pose2d totalDisagreement = this.poseUncertainty.plus(new Pose2d(this.latestVisionXUncertainty, this.latestVisionYUncertainty, this.latestVisionHeadingUncertainty));

            Pose2d kalmanGains = new Pose2d(
                    this.poseUncertainty.vector.x / totalDisagreement.vector.x,
                    this.poseUncertainty.vector.y / totalDisagreement.vector.y,
                    this.poseUncertainty.heading / totalDisagreement.heading
            );

            this.estimatedPose = predictedPose.plus(kalmanGains.times(innovation));
            this.poseUncertainty = this.poseUncertainty.times(new Pose2d(1 / kalmanGains.vector.x, 1 / kalmanGains.vector.y, 1 / kalmanGains.heading));

            this.latestVisionPose = null;
        }
        else {
            this.estimatedPose = predictedPose;
        }
    }
}
