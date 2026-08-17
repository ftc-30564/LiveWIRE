package org.firstinspires.ftc.teamcode.livewire.util.pid.constants;

public class ProfiledControllerConstants {
    public double kMaxAccel;
    public double kMaxDecel;
    public double kMaxVel;

    /**
     * Creates a new ProfiledPIDConstants object with the specified parameters.
     */
    public ProfiledControllerConstants(double kAccel, double kDecel, double kMaxVel) {
        this.kMaxAccel = kAccel;
        this.kMaxDecel = kDecel;
        this.kMaxVel = kMaxVel;
    }
}
