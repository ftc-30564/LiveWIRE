package org.firstinspires.ftc.teamcode.path.points;

public class VelocityPoint {
    public double distance;
    public double velocity;

    /**
     * Creates a new velocity point with the given distance and velocity. This is used to specify a desired velocity at a certain point along the path
     * @param distance The distance of this point along the path, in inches
     * @param velocity The desired velocity at this point, in inches per second
     */
    public VelocityPoint(double distance, double velocity) {
        this.distance = distance;
        this.velocity = velocity;
    }
}
