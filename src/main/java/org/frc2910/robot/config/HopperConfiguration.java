package org.frc2910.robot.config;

import org.frc2910.robot.subsystems.base.roller.RollerMotorSubsystemConfig;

/**
 * Configuration holder for the hopper/feeder subsystem.
 */
public class HopperConfiguration {
    private final RollerMotorSubsystemConfig leftFrontHopperConfig;
    private final RollerMotorSubsystemConfig rightFrontHopperConfig;
    private final RollerMotorSubsystemConfig rightBackHopperConfig;
    private final RollerMotorSubsystemConfig leftBackHopperConfig;

    /**
     * Construct a HopperConfiguration instance.
     *
     * @param leftFrontHopperConfig configuration for the left front hopper roller
     * @param rightFrontHopperConfig configuration for the right front hopper roller
     * @param rightBackHopperConfig configuration for the right back hopper roller
     * @param leftBackHopperConfig configuration for the left back hopper roller
     */
    public HopperConfiguration(
            final RollerMotorSubsystemConfig leftFrontHopperConfig,
            RollerMotorSubsystemConfig rightFrontHopperConfig,
            RollerMotorSubsystemConfig rightBackHopperConfig,
            RollerMotorSubsystemConfig leftBackHopperConfig) {
        this.leftFrontHopperConfig = leftFrontHopperConfig;
        this.rightFrontHopperConfig = rightFrontHopperConfig;
        this.rightBackHopperConfig = rightBackHopperConfig;
        this.leftBackHopperConfig = leftBackHopperConfig;
    }

    /**
     * Get the left-front hopper roller configuration.
     *
     * @return the left front RollerMotorSubsystemConfig
     */
    public RollerMotorSubsystemConfig getLeftFrontHopperConfig() {
        return leftFrontHopperConfig;
    }

    /**
     * Get the right-front hopper roller configuration.
     *
     * @return the right front RollerMotorSubsystemConfig
     */
    public RollerMotorSubsystemConfig getRightFrontHopperConfig() {
        return rightFrontHopperConfig;
    }

    /**
     * Get the right-back hopper roller configuration.
     *
     * @return the right back RollerMotorSubsystemConfig
     */
    public RollerMotorSubsystemConfig getRightBackHopperConfig() {
        return rightBackHopperConfig;
    }

    /**
     * Get the left-back hopper roller configuration.
     *
     * @return the left back RollerMotorSubsystemConfig
     */
    public RollerMotorSubsystemConfig getLeftBackHopperConfig() {
        return leftBackHopperConfig;
    }
}
