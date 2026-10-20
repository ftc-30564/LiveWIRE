package org.firstinspires.ftc.teamcode.livewire.util.math;

public class MathUtil {
    public static boolean withinDeadband(double reference, double actual, double deadband) {
        return Math.abs(reference - actual) <= deadband;
    }
}
