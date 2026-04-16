package org.firstinspires.ftc.teamcode.util.pid;

public class ProfiledPIDConstants {
    public double kV;
    public double kA;
    public double kS;
    public double kP;
    public double kI;
    public double kD;

    public double kMaxAccel;
    public double kMaxDecel;
    public double kMaxVel;

    public double initialVelocity;
    public double finalVelocity;

    /**
     * Creates a new ProfiledPIDConstants object with the specified parameters.
     * @param kV The velocity feedforward constant
     * @param kA The acceleration feedforward constant
     * @param kS The static feedforward constant
     * @param kP The proportional constant
     * @param kI The integral constant
     * @param kD The derivative constant
     */
    public ProfiledPIDConstants(double kV, double kA, double kS, double kP, double kI, double kD,
                               double kAccel, double kDecel, double kMaxVel) {
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.kMaxAccel = kAccel;
        this.kMaxDecel = kDecel;
        this.kMaxVel = kMaxVel;
    }
}
