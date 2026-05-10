package org.firstinspires.ftc.teamcode.util.pid.constants;

public class PIDSVAConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;

    public PIDSVAConstants(double kS, double kV, double kA, double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }
}
