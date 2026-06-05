package org.firstinspires.ftc.teamcode.drivetrain.mecanum;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.pid.constants.PIDSVAConstants;

public class MecanumConstants {
    /**
     * The front left wheel motor hardware map name
     */
    public String frontLeftMotorName;
    /**
     * The front right wheel motor hardware map name
     */
    public String frontRightMotorName;
    /**
     * The back left wheel motor hardware map name
     */
    public String backLeftMotorName;
    /**
     * The back right wheel motor hardware map name
     */
    public String backRightMotorName;

    /**
     * The front left wheel motor direction
     */
    public DcMotorSimple.Direction frontLeftMotorDirection;
    /**
     * The front right wheel motor direction
     */
    public DcMotorSimple.Direction frontRightMotorDirection;
    /**
     * The back left wheel motor direction
     */
    public DcMotorSimple.Direction backLeftMotorDirection;
    /**
     * The back right wheel motor direction
     */
    public DcMotorSimple.Direction backRightMotorDirection;
}
