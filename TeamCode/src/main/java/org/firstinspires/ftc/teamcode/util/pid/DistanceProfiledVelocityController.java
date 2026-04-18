package org.firstinspires.ftc.teamcode.util.pid;

/**
 * Distance based profiled velocity generator. Instead of going based off of time, this goes based off of distance from the target to control
 * deceleration This is meant to react better to disturbances instead of getting thrown off like a time approach would. This purely
 * generates velocity setpoints that are returned. 
 * TODO: this should handle initialVelocity and finalVelocity
 */
public class DistanceProfiledVelocityController {
    private ProfiledControllerConstants constants;

    private double elapsedTimeSeconds = 0;
    private double initialVelocity;
    private double finalVelocity;

    private double timeToAccelerate;
    private double distanceToDecelerate;

    public DistanceProfiledVelocityController(ProfiledControllerConstants constants, double initialVelocity, double finalVelocity) {
        this.constants = constants;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
    }

    public void setConstants(ProfiledControllerConstants constants) {
        this.constants = constants;
    }

    public void reset(double initialVelocity, double finalVelocity) {
        this.elapsedTimeSeconds = 0;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;

        // distances that it takes to accelerate/decelerate from max velocity.
        // TODO: we may never hit max velocity
        this.timeToAccelerate = (constants.kMaxVel - this.initialVelocity) / constants.kMaxAccel;
        this.distanceToDecelerate = (Math.pow(constants.kMaxVel, 2) / (2 * constants.kMaxDecel));
    }

    /**
     * Returns the output of the controller based on the initial, current, and target position. This calculates acceleration and deceleration
     * based on distance from the initial/final. This allows for better handling of disturbances and more accurate following.
     * @param current the current position
     * @param target the target position
     * @param dt time per loop, in ms
     * @return the output of the controller
     */
    public double update(double current, double target, double dt) {
        double error = target - current;
        double ret;

        if (this.elapsedTimeSeconds < this.timeToAccelerate) {
            ret = this.elapsedTimeSeconds * constants.kMaxAccel;
        }
        else if (error < this.distanceToDecelerate) {
            ret = Math.sqrt(2 * constants.kMaxDecel * error);
        }
        else {
            ret = constants.kMaxVel;
        }

        this.elapsedTimeSeconds += dt / 1000;
        return ret;
    }
}
