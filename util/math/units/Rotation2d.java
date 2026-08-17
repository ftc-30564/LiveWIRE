package org.firstinspires.ftc.teamcode.livewire.util.math.units;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Rotation2d {
    private double radians;

    /**
     * Constructs a Rotation2d. This class handles switching between units for rotations, including degrees, radians, and rotations.
     */
    public Rotation2d() {
        this.radians = 0;
    }

    /**
     * Constructs a Rotation2d. This class handles switching between units for rotations, including degrees, radians, and rotations.
     * @param radians initial value, in radians
     */
    private Rotation2d(double radians) {
        this.radians = radians;
    }

    /**
     * Returns a Rotation2d from given degrees
     * @param degrees degrees
     * @return Rotation2d object
     */
    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(degrees / 360.0);
    }

    /**
     * Returns a Rotation2d from given radians
     * @param radians radians
     * @return Rotation2d object
     */
    public static Rotation2d fromRadians(double radians) {
        return new Rotation2d(radians / 2.0 * Math.PI);
    }

    /**
     * Returns a Rotation2d from given rotations
     * @param rotations rotations
     * @return Rotation2d object
     */
    public static Rotation2d fromRotations(double rotations) {
        return new Rotation2d(rotations);
    }

    /**
     * Returns the value in degrees
     * @return degrees
     */
    public double getDegrees() {
        return radians * 180 / Math.PI;
    }

    /**
     * Returns the value in radians
     * @return radians
     */
    public double getRadians() {
        return radians;
    }

    /**
     * Returns the value in rotations (one full rotation is 1)
     * @return rotations
     */
    public double getRotations() {
        return radians / (Math.PI * 2);
    }

    /**
     * Returns the value in the desired unit
     * @param angleUnit desired unit
     * @return value in desired unit
     */
    public double getUnit(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES)
            return getDegrees();
        return getRadians();
    }

    /**
     * Subtracts two rotations, non-wrapping.
     * @param comp the rotation to subtract
     * @return the new rotation
     */
    public Rotation2d minus(Rotation2d comp) {
        return new Rotation2d(this.radians - comp.radians);
    }

    /**
     * Adds two rotations, non-wrapping
     * @param comp the rotation to add
     * @return the new added rotation
     */
    public Rotation2d plus(Rotation2d comp) {
        return new Rotation2d(this.radians + comp.radians);
    }

    /**
     * Returns a new Rotation2d that is within 0 and a full rotation
     * @return the wrapped rotation
     */
    public Rotation2d wrapped() {
        return new Rotation2d(this.radians % (Math.PI * 2));
    }

    /**
     * A string of all the units of the Rotation2d
     * @return string that contains: Rotation2d: Rot: %.3f, Rad: %.3f, Deg: %.3f
     */
    @SuppressLint("DefaultLocale")
    @NonNull
    @Override
    public String toString() {
        return String.format("Rotation2d: Rot: %.3f, Rad: %.3f, Deg: %.3f)", getRotations(), getRadians(), getDegrees());
    }
}
