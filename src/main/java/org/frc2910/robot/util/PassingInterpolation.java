package org.frc2910.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.frc2910.robot.config.Constants;

/**
 * Interpolation table for pass/fuel shooting based on distance.
 *
 * <p>Maps distance to shooter RPM, hood angle (fixed), and estimated time-of-flight
 * to allow synchronized feeding during passing shots.
 * </p>
 */
public class PassingInterpolation {
    InterpolatingDoubleTreeMap RPMInterpolationTable = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap timeInterpolationTable = new InterpolatingDoubleTreeMap(); // RPM to seconds

    public PassingInterpolation() {
        // Passing with overshooting bump
        RPMInterpolationTable.put(2.0, 2000.0);
        RPMInterpolationTable.put(6.0, 3500.0);
        RPMInterpolationTable.put(10.0, 6000.0);

        timeInterpolationTable.put(0.5, 0.5);
        timeInterpolationTable.put(6.0, 0.83);
        timeInterpolationTable.put(10.0, 1.1);
    }

    /**
     * Interpolate the shooter RPM setpoint for a pass at given distance.
     *
     * @param distance the target distance in meters
     * @return the interpolated RPM value
     */
    public double getInterpolatedRPM(double distance) {
        return RPMInterpolationTable.get(distance);
    }

    /**
     * Get the fixed hood angle for passing shots.
     *
     * <p>Currently returns a constant from Constants.ShooterConstants.PASSING_ANGLE.
     * </p>
     *
     * @param distance the target distance (ignored)
     * @return the passing hood angle in degrees
     */
    public double getInterpolatedAngle(double distance) {
        return Constants.ShooterConstants.PASSING_ANGLE;
    }

    /**
     * Interpolate the estimated time-of-flight for a passing shot.
     *
     * <p>Used to help synchronize feeding with the shot flight time.
     * </p>
     *
     * @param distance the target distance in meters
     * @return the estimated time of flight in seconds
     */
    public double getInterpolatedTimeOfFlight(double distance) {
        return timeInterpolationTable.get(distance);
    }
}
