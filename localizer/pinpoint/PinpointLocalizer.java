package org.firstinspires.ftc.teamcode.localizer.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.localizer.Localizer;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Rotation2d;
import org.firstinspires.ftc.teamcode.util.math.units.Distance;

public class PinpointLocalizer implements Localizer {
    private GoBildaPinpointDriver pinpoint;

    // TODO: change these
    public static final String NAME = "pinpoint";
    public static final double FORWARD_OFFSET = 0;
    public static final double STRAFE_OFFSET = 0;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_INVERTED = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_INVERTED = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /**
     * A Gobilda pinpoint localizer.
     * @param hardwareMap hardware map instance
     */
    public PinpointLocalizer(HardwareMap hardwareMap) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointLocalizer.NAME);

        this.pinpoint.setOffsets(PinpointLocalizer.FORWARD_OFFSET, PinpointLocalizer.STRAFE_OFFSET, DistanceUnit.INCH);
        this.pinpoint.setEncoderDirections(PinpointLocalizer.FORWARD_INVERTED, PinpointLocalizer.STRAFE_INVERTED);
        this.pinpoint.setEncoderResolution(PinpointLocalizer.ENCODER_RESOLUTION);
    }

    /**
     * Sets the current pose of the Pinpoint
     * @param pose current pose of the Pinpoint
     */
    @Override
    public void setPose(Pose2d pose) {
        this.pinpoint.setPosition(pose.pose2D());
    }

    /**
     * Returns the current pose from the Pinpoint, as a Pose2d
     * @return current pose from the pinpoint
     */
    @Override
    public Pose2d getPose() {
        return new Pose2d(
                this.pinpoint.getPosition().getX(DistanceUnit.INCH),
                this.pinpoint.getPosition().getY(DistanceUnit.INCH),
                this.pinpoint.getPosition().getHeading(AngleUnit.RADIANS)
        );
    }

    /**
     * Returns the velocity of the pinpoint as a Pose2d
     * @return velocity as a Pose2d.
     */
    @Override
    public Pose2d getVelocity() {
        return new Pose2d(
                this.pinpoint.getVelX(DistanceUnit.INCH),
                this.pinpoint.getVelY(DistanceUnit.INCH),
                this.pinpoint.getHeading(AngleUnit.RADIANS)
                );
    }

    /**
     * Updates the pinpoint
     */
    @Override
    public void update() {
        this.pinpoint.update();
    }
}
