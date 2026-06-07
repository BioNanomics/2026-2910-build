package org.frc2910.robot.subsystems.base.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.frc2910.robot.util.CanDeviceId;

import java.util.ArrayList;
import java.util.List;

/**
 * Configuration class for LEADER motors in RollerWithFollowersSubsystem, contains configuration class for FOLLOWER motors in RollerWithFollowersSubsystem, extends RollerMotorSubsystemConfig
 *
 * @see RollerWithFollowersSubsystem
 * @see RollerMotorSubsystemConfig
 */
public class RollerWithFollowersSubsystemConfig extends RollerMotorSubsystemConfig {
    /**
     * List of follower motor configs for leader motors in RollerWithFollowersSubsystem. Each follower config in the list has to be set.
     *
     * @see RollerWithFollowersSubsystem
     */
    private final List<FollowerConfig> followerConfigs = new ArrayList<>();

    public RollerWithFollowersSubsystemConfig(
            String motorName, CanDeviceId talonID, String mechanismEndUnit, TalonFXConfiguration motorConfiguration) {
        super(motorName, talonID, mechanismEndUnit, motorConfiguration);
    }

    public List<RollerWithFollowersSubsystemConfig.FollowerConfig> getFollowerConfigs() {
        return followerConfigs;
    }

    public void addFollowerConfig(RollerWithFollowersSubsystemConfig.FollowerConfig followerConfig) {
        followerConfigs.add(followerConfig);
    }

    /**
     * Configuration class for FOLLOWER motors in RollerWithFollowersSubsystem.
     *
     * @see RollerWithFollowersSubsystem
     */
    @SuppressWarnings("ClassCanBeStatic")
    public class FollowerConfig extends RollerMotorSubsystemConfig {
        /**
         * Signifies whether the follower motor is inverted or not relative to the leader motor
         */
        private final MotorAlignmentValue motorAlignment;

        /**
         * @param motorAlignment Signifies whether the follower motor is inverted or not relative to the leader motor
         */
        public FollowerConfig(
                String motorName,
                CanDeviceId talonID,
                String mechanismEndUnit,
                TalonFXConfiguration motorConfiguration,
                MotorAlignmentValue motorAlignment) {
            super(motorName, talonID, mechanismEndUnit, motorConfiguration);
            this.motorAlignment = motorAlignment;
        }

        public FollowerConfig withTalonFX(TalonFXConfiguration fxConfig) {
            this.setMotorConfiguration(fxConfig);

            return this;
        }

        /**
         * Get the motor alignment.
         *
         * @return whether the follower motor is inverted or not relative to the leader motor
         */
        public MotorAlignmentValue getMotorAlignment() {
            return motorAlignment;
        }
    }
}
