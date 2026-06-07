package org.frc2910.robot.subsystems.base;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.util.CanDeviceId;

/**
 * Base configuration class for all BaseSubsystem configuration classes (that require motors)
 */
public class BaseMotorConfig {
    /**
     * Name of the motor in relation to the subsystem of this motor and its purpose/location in that subsystem
     */
    private final String motorName;

    private final CanDeviceId talonCANID;

    /**
     * The unit that we are operating the mechanism in with methods like "setMechanismPosition(pos)". This is the unit that the sensorRotationsToMechanismUnitsRatio relates the encoder's rotations to.
     * This is used just to reference what the end unit that you are working with, NOT USED for any Units or conversion purpose.
     */
    private final String mechanismEndUnit;

    private TalonFXConfiguration motorConfiguration;

    /**
     * Create a {@link TalonFXConfiguration} using the <code>sensorRotationsToMechanismUnitsRatio</code> and a default rotor-to-sensor ratio.
     *
     * @param sensorRotationsToMechanismUnitsRatio
     * The sensor rotations to mechanism ratio.
     *
     * @return A {@link TalonFXConfiguration}.
     */
    protected static TalonFXConfiguration createMotorConfiguration(final double sensorRotationsToMechanismUnitsRatio) {
        return new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(sensorRotationsToMechanismUnitsRatio)
                        .withRotorToSensorRatio(Constants.BaseSubsystems.BASE_MOTOR_DEFAULT_ROTOR_TO_SENSOR_RATIO));
    }

    /**
     * @param motorName        Name of the motor in relation to the subsystem of this motor and its purpose/location in that subsystem
     * @param talonID          CanID of the motor
     * @param mechanismEndUnit The unit that we are operating the mechanism in with methods like "setMechanismPosition(pos)". This is the unit that the sensorRotationsToMechanismUnitsRatio relates the encoder's rotations to. This is used just to reference what the end unit that you are working with, NOT USED for any Units or conversion purpose.
     */
    protected BaseMotorConfig(
            final String motorName,
            final CanDeviceId talonID,
            final String mechanismEndUnit,
            final TalonFXConfiguration motorConfiguration) {
        this.motorName = motorName;
        this.talonCANID = talonID;
        this.mechanismEndUnit = mechanismEndUnit;
        this.motorConfiguration = motorConfiguration;
    }

    // Getters
    public String getMotorName() {
        return motorName;
    }

    public CanDeviceId getTalonCANID() {
        return talonCANID;
    }

    public String getMechanismEndUnit() {
        return mechanismEndUnit;
    }

    public TalonFXConfiguration getMotorConfiguration() {
        return motorConfiguration;
    }

    // Simulation configs:
    /**
     * Kg*m^2 of the mechanism. For simulation use only
     */
    private double momentOfInertia = 0.016; // My default. It's for a 1 kg intake with 4cm rad like Thor

    /**
     * Amount of voltage needed to overcome friction (must be positive). For simulation use only
     */
    private double frictionVoltage = 0.25; // 254's default

    // Default deviations to 0.0 so numbers don't change unexpectedly in sim.
    /**
     * Expected error in motor position. For simulation only
     */
    private double positionStandardDeviation = 0.0;

    /**
     * Expected error in motor velocity. For simulation only
     */
    private double velocityStandardDeviation = 0.0;

    public void setSimConfigs(
            double momentOfInertia,
            double frictionVoltage,
            double positionStandardDeviation, // TODO rename to motor or mechanism pos/vel based on if we get dc motor
            double velocityStandardDeviation) {
        this.momentOfInertia = momentOfInertia;
        this.frictionVoltage = frictionVoltage;
        this.positionStandardDeviation = positionStandardDeviation;
        this.velocityStandardDeviation = velocityStandardDeviation;
    }

    public double getMomentOfInertia() {
        return momentOfInertia;
    }

    public double getFrictionVoltage() {
        return frictionVoltage;
    }

    public double getPositionStandardDeviation() {
        return positionStandardDeviation;
    }

    public double getVelocityStandardDeviation() {
        return velocityStandardDeviation;
    }

    protected void setMotorConfiguration(TalonFXConfiguration motorConfiguration) {
        this.motorConfiguration = motorConfiguration;
    }
}
