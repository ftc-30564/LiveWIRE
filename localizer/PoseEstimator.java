package org.firstinspires.ftc.teamcode.livewire.localizer;

import org.firstinspires.ftc.teamcode.livewire.util.math.units.Pose2d;

public class PoseEstimator {
    private final Localizer localizer;

    // the last localizer-estimated pose, used for finding the delta
    private Pose2d odometryDelta;
    private Pose2d poseUncertainty;

    // the total estimated pose
    private Pose2d estimatedPose = new Pose2d();
    // the last estimated pose, used to calculate velocity
    private Pose2d lastEstimatedPose = new Pose2d();
    // the prediction step
    private Pose2d predictedPose;
    private Pose2d velocity = new Pose2d();

    private final Pose2d localizerUncertainty = new Pose2d(0.01, 0.01, 0.01);

    private Pose2d latestVisionPose;
    private Pose2d latestVisionUncertainty;

    private long lastNano = System.nanoTime();

    /**
     * Creates a new PoseEstimator with the specified localizer. The localizer will be used to update the estimated pose of the robot based on the localizer's readings.
     * Additionally, the class also takes in vision measurements, to fuse dead wheel odometry with vision for a more accurate pose estimation by using standard deviations. 
     * @param localizer the localizer to use for updating the estimated pose
     */
    public PoseEstimator(Localizer localizer) {
        this.localizer = localizer;

        // reset the overall pose uncertainty
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
        this.poseUncertainty.zero();
    }

    public void addVisionMeasurement(Pose2d visionPose, Pose2d latestVisionUncertainty) {
        this.latestVisionPose = visionPose;
        this.latestVisionUncertainty = latestVisionUncertainty;
    }

    /**
     * Gets the current field-relative velocity of the robot from the pose estimator, using only odometry (no vision)
     * @return
     */
    public Pose2d getFieldCentricVelocity() {
        return this.velocity;
    }

    /**
     * Updates the estimated pose of the robot based on the localizer's readings. This should be called in a loop to continuously update the pose estimation. 
     */
    public void update() {
        this.lastEstimatedPose = this.estimatedPose;
        // rotate the localizer delta by the current estimated heading
        // TODO: here it should compare with the estimated pose and based on the heading difference, it should rotate the delta
        this.odometryDelta = localizer.getPoseDelta();//new Pose2d(localizer.getPoseDelta().vector, localizer.getPoseDelta().heading);

        // Account for the fact that the pinpoint might have a different heading than the overall estimated pose, since
        // the estimated pose receives vision estimates and the heading might change compared to where the pinpoint thinks
        // it is.
        this.odometryDelta.vector.rotateInPlace(this.estimatedPose.getHeading() - localizer.getPose().heading);

        // the "prediction" step, aka the odometry pose
        this.predictedPose = this.estimatedPose.plus(this.odometryDelta);

        // add to the uncertainty
        this.poseUncertainty.plusInPlace(this.odometryDelta.absInPlace().timesInPlace(this.localizerUncertainty));

        if (false) {//this.latestVisionPose != null) {
            this.latestVisionPose.minusInPlace(this.predictedPose);

            Pose2d totalDisagreement = this.poseUncertainty.plus(this.latestVisionUncertainty);

            Pose2d kalmanGains = new Pose2d(
                    this.poseUncertainty.vector.x / totalDisagreement.vector.x,
                    this.poseUncertainty.vector.y / totalDisagreement.vector.y,
                    this.poseUncertainty.heading / totalDisagreement.heading
            );

            this.estimatedPose = predictedPose.plus(kalmanGains.times(this.latestVisionPose));
            this.poseUncertainty = this.poseUncertainty.times(
                    new Pose2d(
                            1 - kalmanGains.vector.x,
                            1 - kalmanGains.vector.y,
                            1 - kalmanGains.heading));

            this.latestVisionPose = null;
        }
        else {
            this.estimatedPose = predictedPose;
        }

        // oops forgot to divide by delta t
        this.velocity = this.estimatedPose.minus(lastEstimatedPose).divided((System.nanoTime() - this.lastNano) / 1e9);

        this.lastNano = System.nanoTime();
        this.localizer.update();
    }
}
