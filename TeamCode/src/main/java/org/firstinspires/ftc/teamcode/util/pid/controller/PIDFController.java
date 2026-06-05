package org.firstinspires.ftc.teamcode.util.pid.controller;

import org.firstinspires.ftc.teamcode.util.pid.constants.PIDFConstants;

public class PIDFController {
    private PIDFConstants constants;
    private double derivative;
    private double lastError;

    public PIDFController(PIDFConstants constants) {
        this.constants = constants;
    }

    public double update(double error, double dt) {
        this.derivative = (error - lastError) / dt;

        // TODO: integral
        return Math.signum(error) * constants.kF +
                (error * constants.kP) +
                (this.derivative * constants.kD);
    }

    public double getDerivative() {
        return this.derivative;
    }
}
