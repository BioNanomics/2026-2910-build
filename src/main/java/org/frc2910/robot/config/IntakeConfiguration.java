package org.frc2910.robot.config;

import org.frc2910.robot.subsystems.base.roller.RollerMotorSubsystemConfig;
import org.frc2910.robot.subsystems.base.servo.ServoMotorSubsystemConfig;

/**
 * Configuration for the intake subsystem: servo and roller motor configs.
 */
public class IntakeConfiguration {
    private final ServoMotorSubsystemConfig servoConfig;
    private final RollerMotorSubsystemConfig rollerRightConfig;
    private final RollerMotorSubsystemConfig rollerLeftConfig;

    /**
     * Construct an IntakeConfiguration.
     *
     * @param servoConfig configuration for the intake servo
     * @param rollerRightConfig configuration for the right roller motor
     * @param rollerLeftConfig configuration for the left roller motor
     */
    public IntakeConfiguration(
            final ServoMotorSubsystemConfig servoConfig,
            final RollerMotorSubsystemConfig rollerRightConfig,
            final RollerMotorSubsystemConfig rollerLeftConfig) {
        this.servoConfig = servoConfig;
        this.rollerRightConfig = rollerRightConfig;
        this.rollerLeftConfig = rollerLeftConfig;
    }

    /**
     * Get the servo motor subsystem configuration.
     *
     * @return the servo config
     */
    public ServoMotorSubsystemConfig getServoConfig() {
        return servoConfig;
    }

    /**
     * Get the right roller motor configuration.
     *
     * @return the right roller config
     */
    public RollerMotorSubsystemConfig getRollerRightConfig() {
        return rollerRightConfig;
    }

    /**
     * Get the left roller motor configuration.
     *
     * @return the left roller config
     */
    public RollerMotorSubsystemConfig getRollerLeftConfig() {
        return rollerLeftConfig;
    }
}
