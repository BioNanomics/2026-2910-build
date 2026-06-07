package org.frc2910.robot.config;

import org.frc2910.robot.subsystems.base.roller.RollerWithFollowersSubsystemConfig;
import org.frc2910.robot.subsystems.base.servo.ServoMotorSubsystemConfig;

/**
 * Configuration for the shooter subsystem: flywheel and hood motor configs.
 */
public class ShooterConfiguration {
    private final RollerWithFollowersSubsystemConfig leaderRollerConfig;
    private final ServoMotorSubsystemConfig hoodConfig;

    /**
     * Construct a ShooterConfiguration.
     *
     * @param leaderRollerConfig configuration for the flywheel roller motor
     * @param hoodConfig configuration for the hood servo motor
     */
    public ShooterConfiguration(
            final RollerWithFollowersSubsystemConfig leaderRollerConfig, final ServoMotorSubsystemConfig hoodConfig) {
        this.leaderRollerConfig = leaderRollerConfig;
        this.hoodConfig = hoodConfig;
    }

    /**
     * Get the flywheel roller motor configuration.
     *
     * @return the leader roller config
     */
    public RollerWithFollowersSubsystemConfig getLeaderRollerConfig() {
        return leaderRollerConfig;
    }

    /**
     * Get the hood servo motor configuration.
     *
     * @return the hood servo config
     */
    public ServoMotorSubsystemConfig getHoodConfig() {
        return hoodConfig;
    }
}
