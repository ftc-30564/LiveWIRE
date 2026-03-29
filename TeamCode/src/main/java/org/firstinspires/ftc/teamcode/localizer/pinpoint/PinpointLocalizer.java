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
    private PinpointConstants constants;

    /**
     * A Gobilda pinpoint localizer.
     * @param hardwareMap hardware map instance
     * @param constants constants used for the pinpoint
     */
    public PinpointLocalizer(HardwareMap hardwareMap, PinpointConstants constants) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, constants.name);

        this.pinpoint.setOffsets(constants.forwardOffset.inches(), constants.strafeOffset.inches(), DistanceUnit.INCH);
        this.pinpoint.setEncoderDirections(constants.forwardInverted, constants.strafeInverted);
    }

    /**
     * Sets the current pose of the Pinpoint
     * @param pose current pose of the Pinpoint
     */
    @Override
    public void setPose(Pose2d pose) {
        this.pinpoint.setPosition(pose.pose2D(DistanceUnit.INCH, AngleUnit.RADIANS));
    }

    /**
     * Returns the current pose from the Pinpoint, as a Pose2d
     * @return current pose from the pinpoint
     */
    @Override
    public Pose2d getPose() {
        return new Pose2d(
                Distance.fromInches(this.pinpoint.getPosition().getX(DistanceUnit.INCH)),
                Distance.fromInches(this.pinpoint.getPosition().getY(DistanceUnit.INCH)),
                Rotation2d.fromRadians(this.pinpoint.getPosition().getHeading(AngleUnit.RADIANS))
        );
    }

    /**
     * Returns the velocity of the pinpoint as a Pose2d
     * @return velocity as a Pose2d.
     */
    @Override
    public Pose2d getVelocity() {
        return new Pose2d(
                Distance.fromInches(this.pinpoint.getVelX(DistanceUnit.INCH)),
                Distance.fromInches(this.pinpoint.getVelY(DistanceUnit.INCH)),
                Rotation2d.fromRadians(this.pinpoint.getHeading(AngleUnit.RADIANS))
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
