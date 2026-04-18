package org.firstinspires.ftc.teamcode.path.points;

public class CheckPoint {
    public String name;
    public double distance;

    /**
     * Creates a new checkpoint with the given name and distance. This is used to mark a specific point along the path that can be used for 
     * debugging, running certain subsystems, or other purposes.
     * @param name The name of the checkpoint
     * @param distance The distance of the checkpoint along the path, in inches
     */
    public CheckPoint(String name, double distance) {
        this.name = name;
        this.distance = distance;
    }
}
