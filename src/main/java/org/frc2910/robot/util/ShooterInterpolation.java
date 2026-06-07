package org.frc2910.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Interpolation table for shooter tuning based on distance.
 *
 * <p>Maps target distance (in meters) to shooter RPM and hood angle setpoints.
 * Built-in lookup tables allow smooth interpolation between known points.
 * </p>
 */
public class ShooterInterpolation {
    InterpolatingDoubleTreeMap RPMInterpolationTable = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap hoodAngleInterpolationTable = new InterpolatingDoubleTreeMap();
    // distance, rpm/angle

    public ShooterInterpolation() {
        RPMInterpolationTable.put(1.3, 2100.0);
        RPMInterpolationTable.put(2.0, 2250.0);
        RPMInterpolationTable.put(3.0, 2400.0);
        RPMInterpolationTable.put(4.0, 2600.0);
        RPMInterpolationTable.put(5.0, 2900.0);

        hoodAngleInterpolationTable.put(1.3, 0.0);
        hoodAngleInterpolationTable.put(2.0, 8.0);
        hoodAngleInterpolationTable.put(3.0, 17.0);
        hoodAngleInterpolationTable.put(4.0, 23.0);
        hoodAngleInterpolationTable.put(5.0, 28.0);
    }

    /**
     * Interpolate the shooter RPM setpoint for a given distance.
     *
     * @param distance the target distance in meters
     * @return the interpolated RPM value
     */
    public double getInterpolatedRPM(double distance) {
        return RPMInterpolationTable.get(distance);
    }

    /**
     * Interpolate the hood angle setpoint for a given distance.
     *
     * @param distance the target distance in meters
     * @return the interpolated hood angle in degrees
     */
    public double getInterpolatedHoodAngle(double distance) {
        return hoodAngleInterpolationTable.get(distance);
    }
}
