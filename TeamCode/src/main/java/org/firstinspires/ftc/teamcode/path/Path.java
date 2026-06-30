package org.firstinspires.ftc.teamcode.path;

import org.firstinspires.ftc.teamcode.path.points.CheckPoint;
import org.firstinspires.ftc.teamcode.path.points.HeadingPoint;
import org.firstinspires.ftc.teamcode.path.points.Waypoint;
import org.firstinspires.ftc.teamcode.util.math.Line2d;
import org.firstinspires.ftc.teamcode.util.math.units.Rotation2d;
import org.firstinspires.ftc.teamcode.util.math.units.Vector2d;

import java.util.ArrayList;

/**
 * A Path is purely a list of waypoints.
 */
public class Path {
    public ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>() {};

    public Path(Waypoint startingPoint) {
        this.waypoints.add(startingPoint);
    }

    public Path addWaypoint(Waypoint waypoint) {
        this.waypoints.add(waypoint);
        return this;
    }
}