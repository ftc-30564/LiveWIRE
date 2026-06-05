package org.firstinspires.ftc.teamcode.util.pid.controller;

import org.firstinspires.ftc.teamcode.util.pid.constants.ProfiledControllerConstants;

public class DistanceParameterizedVelocityGenerator {
    private ProfiledControllerConstants constants;

    private double initialVelocity;
    private double finalVelocity;

    private double runningSeconds = 0;
    private long lastNanoTime = 0;

    private double timeToAccelerate;
    private double distanceToDecelerate;

    /**
     * Distance based profiled velocity generator. Instead of going based off of purely time, this goes based off of distance from the target to control
     * deceleration. This is meant to react better to disturbances instead of getting thrown off like a time approach would. This purely
     * generates velocity setpoints that are returned. Acceleration is time-based, deceleration is distance-based.
     * TODO: this should handle initialVelocity and finalVelocity
     */
    public DistanceParameterizedVelocityGenerator() {
        this(new ProfiledControllerConstants(0, 0, 0), 0, 0);
    }

    /**
     * Distance based profiled velocity generator. Instead of going based off of purely time, this goes based off of distance from the target to control
     * deceleration. This is meant to react better to disturbances instead of getting thrown off like a time approach would. This purely
     * generates velocity setpoints that are returned. Acceleration is time-based, deceleration is distance-based.
     * TODO: this should handle initialVelocity and finalVelocity
     */
    public DistanceParameterizedVelocityGenerator(ProfiledControllerConstants constants, double initialVelocity, double finalVelocity) {
        this.constants = constants;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
    }

    public void setConstants(ProfiledControllerConstants constants) {
        this.constants = constants;
    }

    public void reset(double initialVelocity, double finalVelocity) {
        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;

        this.runningSeconds = 0;

        // time/distance that it takes to accelerate/decelerate to/from max velocity.
        // TODO: we may never hit max velocity
        this.timeToAccelerate = (constants.kMaxVel - this.initialVelocity) / constants.kMaxAccel;
        this.distanceToDecelerate = (Math.pow(constants.kMaxVel, 2) / (2 * constants.kMaxDecel));
    }

    /**
     * Returns the output of the controller based on the position error. This calculates acceleration and deceleration
     * based on time and distance. This allows for better handling of disturbances and more accurate following.
     * @return the output of the controller
     */
    public double update(double positionError) {
        double dt = System.nanoTime() - this.lastNanoTime;
        this.lastNanoTime = System.nanoTime();

        runningSeconds += (dt / Math.pow(10, 9));

        double ret;

        if (positionError < this.distanceToDecelerate) {
            ret = Math.sqrt(2 * constants.kMaxDecel * positionError);
        }
        else if (runningSeconds < this.timeToAccelerate) {
            ret = runningSeconds * constants.kMaxAccel;
        }
        else {
            ret = constants.kMaxVel;
        }

        return ret;
    }
}
