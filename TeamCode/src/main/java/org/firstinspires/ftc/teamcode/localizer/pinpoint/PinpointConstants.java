package org.firstinspires.ftc.teamcode.localizer.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.teamcode.util.math.units.Distance;

public class PinpointConstants {
    /**
     * The hardware map name of the Pinpoint
     */
    public String name;
    /**
     * The forward (parallel) odometry pod offset from the center of the robot
     */
    public Distance forwardOffset;
    /**
     * The strafe (perpendicular) odometry pod offset from the center of the robot
     */
    public Distance strafeOffset;
    /**
     * The inversion status of the forward (parallel) odometry pod
     */
    public GoBildaPinpointDriver.EncoderDirection forwardInverted;
    /**
     * The inversion status of the strafe (perpendicular) odometry pod
     */
    public GoBildaPinpointDriver.EncoderDirection strafeInverted;
    /**
     * The encoder resolution of the odometry pods
     */
    public GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution;

    /**
     * Creates a new PinpointConstants object with the specified parameters.
     * @param name the hardware map name of the Pinpoint
     * @param forwardOffset the forward (parallel) odometry pod offset from the center of the robot
     * @param strafeOffset the strafe (perpendicular) odometry pod offset from the center of the robot
     * @param forwardInverted the inversion status of the forward (parallel) odometry pod
     * @param strafeInverted the inversion status of the strafe (perpendicular) odometry pod
     * @param encoderResolution the encoder resolution of the odometry pods
     */
    public PinpointConstants(String name,
                             Distance forwardOffset,
                             Distance strafeOffset,
                             GoBildaPinpointDriver.EncoderDirection forwardInverted,
                             GoBildaPinpointDriver.EncoderDirection strafeInverted,
                             GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.name = name;
        this.forwardOffset = forwardOffset;
        this.strafeOffset = strafeOffset;
        this.forwardInverted = forwardInverted;
        this.strafeInverted = strafeInverted;
        this.encoderResolution = encoderResolution;
    }
}
