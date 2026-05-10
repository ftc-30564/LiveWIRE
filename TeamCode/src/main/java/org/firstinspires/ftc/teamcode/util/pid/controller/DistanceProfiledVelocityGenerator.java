package org.firstinspires.ftc.teamcode.util.pid.controller;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.math.units.Distance;
import org.firstinspires.ftc.teamcode.util.pid.constants.ProfiledControllerConstants;

public class DistanceProfiledVelocityGenerator {
    private ProfiledControllerConstants constants;

    private double initialVelocity;
    private double finalVelocity;

    private double timeToAccelerate;
    private double distanceToDecelerate;

    private ElapsedTime elapsedTime = new ElapsedTime();

    public DistanceProfiledVelocityGenerator() {
        this(new ProfiledControllerConstants(0, 0, 0), 0, 0);
    }

    /**
     * Distance based profiled velocity generator. Instead of going based off of purely time, this goes based off of distance from the target to control
     * deceleration. This is meant to react better to disturbances instead of getting thrown off like a time approach would. This purely
     * generates velocity setpoints that are returned. Acceleration is time-based, deceleration is distance-based.
     * TODO: this should handle initialVelocity and finalVelocity
     */
    public DistanceProfiledVelocityGenerator(ProfiledControllerConstants constants, double initialVelocity, double finalVelocity) {
        this.constants = constants;

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
    }

    public void setConstants(ProfiledControllerConstants constants) {
        this.constants = constants;
    }

    public void reset(double initialVelocity, double finalVelocity) {
        this.elapsedTime.reset();

        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;

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
        double ret;

        if (positionError < this.distanceToDecelerate) {
            ret = Math.sqrt(2 * constants.kMaxDecel * positionError);
        }
        else if (this.elapsedTime.seconds() < this.timeToAccelerate) {
            ret = this.elapsedTime.seconds() * constants.kMaxAccel;
        }
        else {
            ret = constants.kMaxVel;
        }

        return ret;
    }
}
