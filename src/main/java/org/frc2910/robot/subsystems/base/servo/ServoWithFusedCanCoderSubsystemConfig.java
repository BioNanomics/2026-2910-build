package org.frc2910.robot.subsystems.base.servo;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.frc2910.robot.util.CanDeviceId;

/**
 * Configuration class for ServoWithFusedCanCoderSubsystem, extends ServoMotorSubsystemConfig.
 *
 * @see ServoWithFusedCanCoderSubsystem
 * @see ServoMotorSubsystemConfig
 */
public class ServoWithFusedCanCoderSubsystemConfig extends ServoMotorSubsystemConfig {
    private final CanDeviceId canCoderCanID;
    private CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    public ServoWithFusedCanCoderSubsystemConfig(
            String motorName,
            CanDeviceId talonID,
            CanDeviceId canCoderID,
            String mechanismEndUnit,
            TalonFXConfiguration motorConfiguration) {
        super(motorName, talonID, mechanismEndUnit, motorConfiguration);
        this.canCoderCanID = canCoderID;
    }

    public CanDeviceId getCanCoderID() {
        return canCoderCanID;
    }

    public CANcoderConfiguration getCanCoderConfig() {
        return canCoderConfig;
    }
}
