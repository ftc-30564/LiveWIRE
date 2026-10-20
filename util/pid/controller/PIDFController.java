package org.firstinspires.ftc.teamcode.livewire.util.pid.controller;

import org.firstinspires.ftc.teamcode.livewire.util.pid.constants.PIDFConstants;

public class PIDFController {
    private PIDFConstants constants;
    private double derivative;
    private double lastError;
    private long lastNanoTime;

    public PIDFController(PIDFConstants constants) {
        this.constants = constants;
    }

    public void setConstants(PIDFConstants constants) {
        this.constants = constants;
    }

    public double update(double error) {
        long dt = System.nanoTime() - lastNanoTime;
        this.lastNanoTime = System.nanoTime();

        this.derivative = (error - lastError) / (dt / 10e9);

        this.lastError = error;

        // TODO: integral
        return Math.signum(error) * constants.kF +
                (error * constants.kP) +
                (this.derivative * constants.kD);
    }

    public double getDerivative() {
        return this.derivative;
    }
}
