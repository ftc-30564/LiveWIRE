package org.firstinspires.ftc.teamcode.util.pid.constants;

public class PIDSVAConstants {
    public double kS;
    public double kV;
    public double kA;
    public double kP;
    public double kI;
    public double kD;

    public PIDSVAConstants(double kS, double kV, double kA, double kP, double kI, double kD) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
