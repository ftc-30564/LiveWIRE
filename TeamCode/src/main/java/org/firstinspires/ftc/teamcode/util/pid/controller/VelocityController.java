package org.firstinspires.ftc.teamcode.util.pid.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.pid.constants.PIDSVAConstants;

public class VelocityController {
    private PIDSVAConstants constants;
    private long lastNanoTime = System.nanoTime();
    private double lastSetpoint = 0;
    private double lastVelocity = 0;


    /**
     * A velocity controller class. Handles all the math to keep a mechanism at a desired velocity.
     * This takes advantage of kS, kV, and kA for feedforward, and kP, kI, and kD for feedback.
     * @param constants the PIDSVA constants
     */
    public VelocityController(PIDSVAConstants constants) {
        this.constants = constants;
    }

    public double update(double current, double setpoint) {
        double dt = System.nanoTime() - lastNanoTime;

        this.lastNanoTime = System.nanoTime();

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
