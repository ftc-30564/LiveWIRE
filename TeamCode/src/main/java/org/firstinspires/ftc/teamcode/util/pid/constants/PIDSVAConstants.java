package org.firstinspires.ftc.teamcode.util.pid.constants;

public class PIDSVAConstants {
    public double kP;
    public double kI;
    public double kD;
    public double kS;
    public double kV;
    public double kA;

    public PIDSVAConstants() {
        this(0, 0, 0, 0, 0, 0);
    }

    public PIDSVAConstants(double kS, double kV, double kA, double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Weights the current constants with a set of other constants
     * @param other the other constants
     * @param percent the percent the other constants will take in the average
     * @return new constants
     */
    public PIDSVAConstants weightedAverage(PIDSVAConstants other, double percent) {
        PIDSVAConstants ret = new PIDSVAConstants();

        ret.kP = (this.kP * (1-percent)) + (other.kP * percent);
        ret.kI = (this.kI * (1-percent)) + (other.kI * percent);
        ret.kD = (this.kD * (1-percent)) + (other.kD * percent);
        ret.kS = (this.kS * (1-percent)) + (other.kS * percent);
        ret.kV = (this.kV * (1-percent)) + (other.kV * percent);
        ret.kA = (this.kA * (1-percent)) + (other.kA * percent);

        return ret;
    }
}
