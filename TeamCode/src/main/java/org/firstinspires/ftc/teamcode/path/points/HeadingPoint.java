package org.firstinspires.ftc.teamcode.path.points;

import org.firstinspires.ftc.teamcode.util.math.units.Rotation2d;

public class HeadingPoint {
    public Rotation2d heading;
    public double distance;

    public HeadingPoint(Rotation2d heading, double distance) {
        this.heading = heading;
        this.distance = distance;
    }
}
