package org.frc2910.robot.subsystems.base.servo;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.frc2910.robot.util.CanDeviceId;

import java.util.ArrayList;
import java.util.List;

/**
 * Configuration class for LEADER motors in ServoWithFollowersSubsystem, extends ServoMotorSubsystemConfig
 *
 * @see ServoWithFollowersSubsystem
 * @see ServoMotorSubsystemConfig
 */
public class ServoWithFollowersSubsystemConfig extends ServoMotorSubsystemConfig {
    /**
     * List of follower motor configs for leader motors in ServoWithFollowersSubsystem. Each follower config in the list has to be set.
     *
     * @see ServoWithFollowersSubsystem
     */
    private List<FollowerConfig> followerConfigs = new ArrayList<>();

    public ServoWithFollowersSubsystemConfig(
            String motorName, CanDeviceId talonID, String mechanismEndUnit, TalonFXConfiguration motorConfiguration) {
        super(motorName, talonID, mechanismEndUnit, motorConfiguration);
    }

    public List<FollowerConfig> getFollowerConfigs() {
        return followerConfigs;
    }

    public void addFollowerConfig(FollowerConfig followerConfig) {
        followerConfigs.add(followerConfig);
    }

    /**
     * Configuration class for FOLLOWER motors in ServoWithFollowersSubsystem.
     *
     * @see ServoWithFollowersSubsystem
     */
    @SuppressWarnings("ClassCanBeStatic")
    public class FollowerConfig extends ServoMotorSubsystemConfig {
        /**
         * Signifies whether the follower motor is inverted or not relative to the leader motor
         */
        private final MotorAlignmentValue motorAlignment;

        /**
         * @param motorAlignment Signifies whether the follower motor is inverted or not relative to the leader motor
         * @apiNote The FollowerConfig uses the max/min pos of infinities, documentation says follower talon follows "motor output" of the leader. In voltage, it follows the input voltage but that cuts off if past the leader's max/min pos. In dutyCycle, it follows dutyCycle percentage but that is what we want for homing. Thus, we don't seem to need Follower max/min pos as a param.
         */
        public FollowerConfig( // TODO test whether the servo follower properly follows with new constructor change
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
