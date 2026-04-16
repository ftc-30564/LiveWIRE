package org.firstinspires.ftc.teamcode.util.pid;

/**
 * Distance based profiled PID controller. Instead of going based off of time, this goes based off of distance from the target to control
 * deceleration This is meant to react better to disturbances instead of getting thrown off like a time approach would. This combines
 * feedforward + PID for optimal following. TODO: this should handle initialVelocity and finalVelocity
 */
public class DistanceProfiledPIDController {
    private ProfiledPIDConstants constants;

    private double elapsedTime = 0;
    private double currentVelocity = 0;
    private double previousVelocity = 0;
    private double targetVelocity = 0;
    private double previousTargetVelocity = 0;
    private double previousError = 0;
    private double initialVelocity = 0;
    private double finalVelocity = 0;

    private double distanceToAccelerate;
    private double distanceToDecelerate;

    public DistanceProfiledPIDController(ProfiledPIDConstants constants, double initialVelocity, double finalVelocity) {
        this.constants = constants;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
    }

    public void setConstants(ProfiledPIDConstants constants) {
        this.constants = constants;
    }

    public void reset(double initialVelocity, double finalVelocity) {
        this.elapsedTime = 0;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
    }

    /**
     * Returns the output of the controller based on the initial, current, and target position. This calculates acceleration and deceleration
     * based on distance from the initial/final. This allows for better handling of disturbances and more accurate following.
     * @param initial the initial position
     * @param current the current position
     * @param target the target position
     * @param dt time per loop
     * @return the output of the controller
     */
    public double update(double initial, double current, double target, double dt) {
        double error = target - current;

        this.currentVelocity = (error - previousError) / dt;

        // distances that it takes to accelerate/decelerate from max velocity.
        // TODO: we may never hit max velocity
        this.distanceToAccelerate = (Math.pow(constants.kMaxVel, 2) / (2 * constants.kMaxDecel));
        this.distanceToDecelerate = (Math.pow(constants.kMaxVel, 2) / (2 * constants.kMaxDecel));

        if ((current - initial) < this.distanceToAccelerate) {
            targetVelocity = Math.sqrt(2 * constants.kMaxDecel * (current - initial));
        }
        else if (error < this.distanceToDecelerate) {
            targetVelocity = Math.sqrt(2 * constants.kMaxDecel * error);
        }
        else {
            targetVelocity = constants.kMaxVel;
        }

        // TODO: integral
        double ret = constants.kV * targetVelocity + 
                constants.kA * (targetVelocity - previousTargetVelocity) / dt + 
                constants.kS * Math.signum(targetVelocity) + 
                constants.kP * (targetVelocity - currentVelocity) +
                constants.kD * (currentVelocity - previousVelocity) / dt;
        
        previousTargetVelocity = targetVelocity;
        previousVelocity = currentVelocity;
        previousError = error;

        return ret;
    }
}
