package org.firstinspires.ftc.teamcode.util.pid;

public class ProfiledControllerConstants {
//    public double kV;
//    public double kA;
//    public double kS;
//    public double kP;
//    public double kI;
//    public double kD;

    public double kMaxAccel;
    public double kMaxDecel;
    public double kMaxVel;

    /**
     * Creates a new ProfiledPIDConstants object with the specified parameters.
     */
    public ProfiledControllerConstants(double kAccel, double kDecel, double kMaxVel) {
        this.kMaxAccel = kAccel;
        this.kMaxDecel = kDecel;
        this.kMaxVel = kMaxVel;
    }
}
