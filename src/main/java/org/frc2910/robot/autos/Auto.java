package org.frc2910.robot.autos;

/**
 * Enumeration of autonomous programs supported by the robot.
 *
 * <p>Each enum value corresponds to a program that can be selected via the
 * {@link org.frc2910.robot.autos.AutoChooser} and constructed by
 * {@link org.frc2910.robot.autos.AutoFactory}.
 * </p>
 */
public enum Auto {
    IDLE,
    TEST,

    LEFT_NEUTRAL_ZONE,
    RIGHT_NEUTRAL_ZONE,

    LEFT_NEUTRAL_ZONE_TO_DEPOT,
    DEPOT,

    HOMING,

    // System characterization
    SYSID_TRANSLATION,
    SYSID_ROTATION,
    SYSID_STEER,
}
