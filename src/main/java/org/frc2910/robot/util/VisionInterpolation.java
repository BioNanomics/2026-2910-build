package org.frc2910.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * Interpolation table for vision-based distance estimation.
 *
 * <p>Maps measured pixel distance (from camera) to actual field distance (in meters)
 * using calibrated points from AprilTag sightings at various distances.
 * </p>
 */
public class VisionInterpolation {
    private final InterpolatingDoubleTreeMap distanceFromAprilTag = new InterpolatingDoubleTreeMap();

    public VisionInterpolation() {
        distanceFromAprilTag.put(113.0, Units.inchesToMeters(29.0));
        distanceFromAprilTag.put(101.7, Units.inchesToMeters(48.0));
        distanceFromAprilTag.put(79.2, Units.inchesToMeters(72.0));
        distanceFromAprilTag.put(63.8, Units.inchesToMeters(96.0));
        distanceFromAprilTag.put(51.0, Units.inchesToMeters(120.0));
        distanceFromAprilTag.put(43.5, Units.inchesToMeters(144.0));
        distanceFromAprilTag.put(38.0, Units.inchesToMeters(168.0));
        distanceFromAprilTag.put(33.8, Units.inchesToMeters(192.0));
        distanceFromAprilTag.put(29.1, Units.inchesToMeters(216.0));
        distanceFromAprilTag.put(25.8, Units.inchesToMeters(240.0));
    }

    /**
     * Interpolate the actual distance from a measured pixel distance.
     *
     * @param distance the measured distance in pixels from the camera to the AprilTag
     * @return the interpolated actual distance in meters
     */
    public double getInterpolatedScaledDistance(double distance) {
        return distanceFromAprilTag.get(distance);
    }
}
