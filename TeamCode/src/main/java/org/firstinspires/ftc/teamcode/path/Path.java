package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.path.points.CheckPoint;
import org.firstinspires.ftc.teamcode.path.points.HeadingPoint;
import org.firstinspires.ftc.teamcode.path.points.VelocityPoint;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.units.Pose2d;
import org.firstinspires.ftc.teamcode.util.math.units.Rotation2d;

import java.util.ArrayList;

public class Path {
    public ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>() {};
    public ArrayList<HeadingPoint> headingPoints = new ArrayList<HeadingPoint>() {};
    public ArrayList<VelocityPoint> velocityPoints = new ArrayList<VelocityPoint>() {};
    public ArrayList<CheckPoint> checkPoints = new ArrayList<CheckPoint>();

    public double maxAccel;
    public double maxDecel;
    public double maxVelocity;

    public Path(Waypoint startingPoint) {
        this.waypoints.add(startingPoint);
    }

    public Path addWaypoint(Waypoint waypoint) {
        this.waypoints.add(waypoint);
        return this;
    }

    public Path addHeadingPoint(Rotation2d heading, double distance) {
        this.headingPoints.add(new HeadingPoint(heading, distance));
        return this;
    }

    public Path addVelocityPoint(double velocity, double distance) {
        this.velocityPoints.add(new VelocityPoint(velocity, distance));
        return this;
    }

    public Path addCheckPoint(String name, double distance) {
        this.checkPoints.add(new CheckPoint(name, distance));
        return this;
    }

    public Path withMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    public Path withMaxDecel(double maxDecel) {
        this.maxDecel = maxDecel;
        return this;
    }

    public Path withMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }
}