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
     */
    public DistanceParameterizedVelocityGenerator() {
        this(new ProfiledControllerConstants(0, 0, 0), 0, 0);
    }

    /**
     * Distance based profiled velocity generator. Instead of going based off of purely time, this goes based off of distance from the target to control
     * deceleration. This is meant to react better to disturbances instead of getting thrown off like a time approach would. This purely
     * generates velocity setpoints that are returned. Acceleration is time-based, deceleration is distance-based.
     */
    public DistanceParameterizedVelocityGenerator(ProfiledControllerConstants constants, double initialVelocity, double finalVelocity) {
        this.constants = constants;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
    }

    public void setConstants(ProfiledControllerConstants constants) {
        this.constants = constants;
    }

    public void reset(double initialVelocity, double finalVelocity, double totalDistance) {
        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;

        this.runningSeconds = 0;

        // time/distance that it takes to accelerate/decelerate to/from max velocity.
        // TODO: we may never hit max velocity
        // if the distance it takes to accelerate AND decelerate add up to greater than the distance between the two waypoints,
        // figure out the max speed the robot can accelerate to before having to decelerate

        //this.timeToAccelerate = (constants.kMaxVel - this.initialVelocity) / constants.kMaxAccel;
        // d = vt * .5at^2
        //double distanceToAccelerate = (this.initialVelocity * this.timeToAccelerate) + (.5 * constants.kMaxAccel * (Math.pow(this.timeToAccelerate, 2)));

        // d = v^2 / 2a
        //this.distanceToDecelerate = (Math.pow(constants.kMaxVel, 2) / (2 * constants.kMaxDecel));
    }

    /**
     * Returns the output of the controller based on the position error. This calculates acceleration and deceleration
     * based on time and distance. This allows for better handling of disturbances and more accurate following.
     * @return the output of the controller
     */
    public double update(double currentVelocity, double distance) {
        double dt = System.nanoTime() - this.lastNanoTime;
        this.lastNanoTime = System.nanoTime();

//        if (distance < this.distanceToDecelerate) {
//            ret = Math.sqrt(2 * constants.kMaxDecel * distance);
//        }
//        else if (runningSeconds < this.timeToAccelerate) {
//            ret = runningSeconds * constants.kMaxAccel;
//        }
//        else {
//            ret = constants.kMaxVel;
//        }

        return Math.min(Math.min(
                currentVelocity + (constants.kMaxAccel * dt),
                constants.kMaxVel),
                Math.sqrt(Math.pow(this.finalVelocity, 2) + (2 * constants.kMaxDecel * distance))
        );
    }
}
