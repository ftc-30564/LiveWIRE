package org.firstinspires.ftc.teamcode.path.points;

import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;

public class Waypoint {
    public Pose2d point;
    public double maxAccel;
    public double maxDecel;
    public double velocity = 0;

    public Waypoint(Pose2d point, double maxAccel, double maxDecel) {
        this.point = point;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
    }
}
