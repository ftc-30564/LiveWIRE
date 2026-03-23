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
}
