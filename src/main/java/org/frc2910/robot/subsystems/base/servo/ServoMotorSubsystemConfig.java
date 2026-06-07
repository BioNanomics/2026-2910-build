package org.frc2910.robot.subsystems.base.servo;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.frc2910.robot.subsystems.base.BaseMotorConfig;
import org.frc2910.robot.util.CanDeviceId;

/**
 * Configuration class for ServoMotorSubsystem, extends BaseMotorConfig
 *
 * @see ServoMotorSubsystem
 * @see BaseMotorConfig
 */
public class ServoMotorSubsystemConfig extends BaseMotorConfig {
    /**
     * Create a {@link TalonFXConfiguration} for the servo
     * using the <code>sensorRotationsToMechanismUnitsRatio</code> and a default rotor-to-sensor ratio
     * and sets the soft limit thresholds.
     * Motor neutral mode is set to <code>NeutralModeValue.Break</code>.
     *
     * @param sensorRotationsToMechanismUnitsRatio
     * The sensor rotations to mechanism ratio.
     *
     * @param reverseSoftLimitThreshold
     * The reverse soft limit threshold.
     *
     * @param forwardSoftLimitThreshold
     * The forward soft limit threshold.
     *
     * @return A {@link TalonFXConfiguration}.
     */
    public static TalonFXConfiguration createMotorConfiguration(
            final double sensorRotationsToMechanismUnitsRatio,
            final double reverseSoftLimitThreshold,
            final double forwardSoftLimitThreshold) {
        return BaseMotorConfig.createMotorConfiguration(sensorRotationsToMechanismUnitsRatio)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(forwardSoftLimitThreshold)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(reverseSoftLimitThreshold))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    }

    public ServoMotorSubsystemConfig(
            String motorName,
            final CanDeviceId talonID,
            final String mechanismEndUnit,
            final TalonFXConfiguration motorConfiguration) {
        super(motorName, talonID, mechanismEndUnit, motorConfiguration);
    }
}
