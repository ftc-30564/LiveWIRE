package org.firstinspires.ftc.teamcode.util.math.units;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance {
    private double inches;

    public Distance() {
        this(0);
    }

    /**
     * A distance measurement. This class handles switching between different units, such as in, m, cm, and mm seamlessly.
     * @param inches measurement in inches
     */
    public Distance(double inches) {
        this.inches = inches;
    }

    /**
     * A distance measurement. This class handles switching between different units, such as in, m, cm, and mm seamlessly.
     * @param amt Measurement, in the specified unit
     * @param unit Unit that the amount is in.
     */
    public Distance(double amt, DistanceUnit unit) {
        if (unit == DistanceUnit.INCH)
            this.inches = amt;
        if (unit == DistanceUnit.METER)
            this.inches = metersToInches(amt);
        if (unit == DistanceUnit.CM)
            this.inches = metersToInches(amt / 100);
        if (unit == DistanceUnit.MM)
            this.inches = metersToInches(amt / 1000);
    }

    /**
     * Returns the current distance as meters.
     * @return distance in meters
     */
    public double meters() {
        return this.inches * 0.0254;
    }

    /**
     * Returns the current distance as inches
     * @return distance in inches
     */
    public double inches() {
        return this.inches;
    }

    /**
     * Adds 2 distances
     * @param distance the distance to add
     * @return the combined distance
     */
    public Distance plus(Distance distance) {
        return fromInches(this.inches() + distance.inches());
    }

    /**
     * Subtracts 2 distances
     * @param distance the distance to subtract
     * @return the new distance
     */
    public Distance minus(Distance distance) {
        return fromInches(this.inches() - distance.inches());
    }

    /**
     * Returns the distance in a specified unit
     * @param unit the unit to return
     * @return distance in a unit
     */
    public double unit(DistanceUnit unit) {
        if (unit == DistanceUnit.INCH)
            return inches();
        if (unit == DistanceUnit.CM)
            return meters() * 100;
        if (unit == DistanceUnit.MM)
            return meters() * 1000;
        return meters();
    }

    /**
     * Converts given inches into meters
     * @param inches inches
     * @return new distance in meters
     */
    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    /**
     * Converts the given meters into inches
     * @param meters meters
     * @return new distance in inches
     */
    public static double metersToInches(double meters) {
        return meters * 39.3701;
    }

    /**
     * Converts a given unit into another unit
     * @param amt the number to convert
     * @param oldUnit unit that the number is currently in
     * @param newUnit the desired unit
     * @return the converted number
     */
    public static double unitToUnit(double amt, DistanceUnit oldUnit, DistanceUnit newUnit) {
        if (newUnit == DistanceUnit.INCH)
            return oldUnit.toInches(amt);
        if (newUnit == DistanceUnit.METER)
            return oldUnit.toMeters(amt);
        if (newUnit == DistanceUnit.MM)
            return oldUnit.toMm(amt);
        return oldUnit.toCm(amt);
    }

    /**
     * Creates a new Distance object from inches
     * @param inches amount in inches
     * @return Distance object
     */
    public static Distance fromInches(double inches) {
        return new Distance(inches);
    }

    /**
     * Creates a new Distance object from meters
     * @param meters amount in meters
     * @return Distance object
     */
    public static Distance fromMeters(double meters) {
        return new Distance(metersToInches(meters));
    }

    public static Distance fromCm(double cm) {
        return new Distance(metersToInches(cm / 100));
    }

    public static Distance fromMm(double mm) {
        return new Distance(metersToInches(mm / 1000));
    }

    /**
     * Creates a new Distance object from a specified unit
     * @param amt numerical amount in unit
     * @param unit unit that the amount is in
     * @return a Distance object
     */
    public static Distance fromUnit(double amt, DistanceUnit unit) {
        if (unit == DistanceUnit.INCH) {
            return fromInches(amt);
        }
        return fromMeters(amt);
    }
}