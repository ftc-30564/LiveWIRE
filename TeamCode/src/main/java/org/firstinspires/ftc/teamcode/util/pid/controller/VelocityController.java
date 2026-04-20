package org.firstinspires.ftc.teamcode.util.pid.controller;

import org.firstinspires.ftc.teamcode.util.pid.constants.PIDSVAConstants;

public class VelocityController {
    private PIDSVAConstants constants;
    private double lastSetpoint = 0;
    private double lastVelocity = 0;

    /**
     * A velocity controller class. Handles all the math. // TODO: integral
     * @param constants
     */
    public VelocityController(PIDSVAConstants constants) {
        this.constants = constants;
    }

    public double update(double setpoint, double current, double dt) {
        double targetAccel = (setpoint - lastSetpoint) / dt;
        double actualAccel = (current - lastVelocity) / dt;
        double error = setpoint - current;

        lastSetpoint = setpoint;
        lastVelocity = current;

        return Math.signum(setpoint) * constants.kS +
                setpoint * constants.kV +
                targetAccel * constants.kA +
                (error * constants.kP) +
                (actualAccel * constants.kD);

    }
}
