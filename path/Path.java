package org.firstinspires.ftc.teamcode.livewire.path;

import org.firstinspires.ftc.teamcode.livewire.path.points.Waypoint;

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