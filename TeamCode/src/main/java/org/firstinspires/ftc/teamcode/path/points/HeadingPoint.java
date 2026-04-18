package org.firstinspires.ftc.teamcode.path.points;

import org.firstinspires.ftc.teamcode.util.math.units.Rotation2d;

public class HeadingPoint {
    public Rotation2d heading;
    public double distance;

    /**
     * Creates a new heading point with the given heading and distance. This is used to specify a desired heading at a certain point along the path
     * @param heading The desired heading at this point, in radians
     * @param distance The distance of this point along the path, in inches
     */
    public HeadingPoint(Rotation2d heading, double distance) {
        this.heading = heading;
        this.distance = distance;
    }
}
