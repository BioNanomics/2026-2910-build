package org.frc2910.robot.subsystems.base.roller;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.frc2910.robot.subsystems.base.BaseMotorConfig;
import org.frc2910.robot.util.CanDeviceId;

/**
 * Configuration class for RollerMotorSubsystem, extends BaseMotorConfig
 *
 * @see RollerMotorSubsystem
 * @see BaseMotorConfig
 */
public class RollerMotorSubsystemConfig extends BaseMotorConfig {
    /**
     * Create a {@link TalonFXConfiguration} for the servo
     * using the <code>sensorRotationsToMechanismUnitsRatio</code> and a default rotor-to-sensor ratio.
     * Motor neutral mode is set to <code>NeutralModeValue.Coast</code>.
     *
     * @param sensorRotationsToMechanismUnitsRatio
     * The sensor rotations to mechanism ratio.
     *
     * @return A {@link TalonFXConfiguration}.
     */
    public static TalonFXConfiguration createMotorConfiguration(final double sensorRotationsToMechanismUnitsRatio) {
        return BaseMotorConfig.createMotorConfiguration(sensorRotationsToMechanismUnitsRatio)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(false)
                        .withReverseSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(Double.POSITIVE_INFINITY)
                        .withReverseSoftLimitThreshold(Double.NEGATIVE_INFINITY))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }

    /**
     * Defaults: neutral mode to coast and disables Soft-Limits and sets them to POS/NEG infinity
     */
    public RollerMotorSubsystemConfig(
            String motorName, CanDeviceId talonID, String mechanismEndUnit, TalonFXConfiguration motorConfiguration) {
        super(motorName, talonID, mechanismEndUnit, motorConfiguration);
    }
}
