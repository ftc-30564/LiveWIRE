package org.firstinspires.ftc.teamcode.livewire.util.pid.constants;

public class PIDFConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    public PIDFConstants(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}
