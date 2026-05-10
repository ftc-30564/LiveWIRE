package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.path.points.CheckPoint;
import org.firstinspires.ftc.teamcode.path.points.HeadingPoint;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.units.Rotation2d;

import java.util.ArrayList;

public class Path {
    public ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>() {};
    public ArrayList<HeadingPoint> headingPoints = new ArrayList<HeadingPoint>() {};
    public ArrayList<CheckPoint> checkPoints = new ArrayList<CheckPoint>();

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

    public Path addCheckPoint(String name, double distance) {
        this.checkPoints.add(new CheckPoint(name, distance));
        return this;
    }
}