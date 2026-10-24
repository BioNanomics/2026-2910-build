package org.frc2910.robot.subsystems.base;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import org.frc2910.robot.util.CanDeviceId;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.StatusSignalRefresher;

/*
 DutyCycleOut and VoltageOut are open loop control (based on the motor and don't use feedback for a certain output)
 PositionVoltage, VelocityVoltage, and MotionMagic are closed loop control and use the rotor-sensor and sensor-mech
 ratios so they reason in mechanism units
 getRotorPos/Vel() is for motor, everything else like getPos/Vel() is in end mech units
 Default slot for setControl is slot 0 configs
*/

/**
 * TalonFX implementation of MotorIO used by all REAL BaseSubsystems
 *
 * @see MotorIO
 */
public class MotorIOTalonFX implements MotorIO {
    protected final TalonFX talon;
    protected final BaseMotorConfig config;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0).withEnableFOC(true);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0).withEnableFOC(true);
    private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage =
            new DynamicMotionMagicVoltage(0.0, 0.0, 0.0).withEnableFOC(true);
    private final Follower followerControl = new Follower(0, MotorAlignmentValue.Aligned);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> rawRotorPositionSignal;
    private final StatusSignal<AngularVelocity> rotorVelocitySignal;
    private final StatusSignal<Current> currentStatorSignal;
    private final StatusSignal<Current> currentSupplySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Angle> rawMechanismPositionSignal;
    private final StatusSignal<AngularVelocity> mechanismVelocitySignal;
    private final StatusSignal<AngularAcceleration> mechanismAccelerationSignal;
    // These are needed for followers along with motor voltage
    private final StatusSignal<Double> dutyCycleVoltageSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final ConditionalCallbackManager callbackManager;

    protected final double motorRotationsToMechanismUnitsRatio;
    protected final String mechanismEndUnit;

    public MotorIOTalonFX(BaseMotorConfig config, ConditionalCallbackManager callbackManager) {
        this.config = config;
        talon = new TalonFX(
                config.getTalonCANID().getDeviceNumber(), config.getTalonCANID().getBus());
        this.mechanismEndUnit = config.getMechanismEndUnit();
        this.callbackManager = callbackManager;

        talon.getConfigurator().apply(config.getMotorConfiguration());
        rawRotorPositionSignal = talon.getRotorPosition();
        rotorVelocitySignal = talon.getRotorVelocity();
        voltageSignal = talon.getMotorVoltage();
        currentStatorSignal = talon.getStatorCurrent();
        currentSupplySignal = talon.getSupplyCurrent();
        temperatureSignal = talon.getDeviceTemp();
        rawMechanismPositionSignal = talon.getPosition();
        mechanismVelocitySignal = talon.getVelocity();
        mechanismAccelerationSignal = talon.getAcceleration();
        dutyCycleVoltageSignal = talon.getDutyCycle();
        torqueCurrentSignal = talon.getTorqueCurrent();

        callbackManager.registerOneTimeCallback(() -> {
            StatusSignalRefresher.getInstance()
                    .addStatusSignals(
                            config.getTalonCANID().getBus(),
                            rawRotorPositionSignal,
                            voltageSignal,
                            currentStatorSignal,
                            currentSupplySignal,
                            temperatureSignal,
                            mechanismAccelerationSignal,
                            dutyCycleVoltageSignal,
                            torqueCurrentSignal);
        });
        StatusSignalRefresher.getInstance()
                .addStatusSignals(
                        config.getTalonCANID().getBus(),
                        rotorVelocitySignal,
                        rawMechanismPositionSignal,
                        mechanismVelocitySignal);

        // Set a default update frequency for all signals

        // Add all signals to the collection for easy refreshing in refreshData()
        signals.addSignals(
                rawRotorPositionSignal,
                rotorVelocitySignal,
                voltageSignal,
                currentStatorSignal,
                currentSupplySignal,
                temperatureSignal,
                rawMechanismPositionSignal,
                mechanismVelocitySignal,
                mechanismAccelerationSignal,
                dutyCycleVoltageSignal,
                torqueCurrentSignal);
        signals.setUpdateFrequencyForAll(100.0); // 100 hz

        // Optimizes the device's bus utilization by reducing the update frequencies
        // of its status signals. This API defaults to an optimized update frequency
        // of 4 Hz to preserve log data.
        // All status signals that have not been explicitly given an update frequency
        // using BaseStatusSignal.setUpdateFrequency will be slowed down.
        talon.optimizeBusUtilization();

        this.motorRotationsToMechanismUnitsRatio = config.getMotorConfiguration().Feedback.RotorToSensorRatio
                * config.getMotorConfiguration().Feedback.SensorToMechanismRatio;
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        callbackManager.callIfEnabled(() -> {
            inputs.motorRawPositionRotations = rawRotorPositionSignal.getValueAsDouble();
            inputs.motorStatorCurrentAmps = currentStatorSignal.getValueAsDouble();
            inputs.motorSupplyCurrentAmps = currentSupplySignal.getValueAsDouble();
            inputs.motorVoltage = voltageSignal.getValueAsDouble();
            inputs.motorTemperatureC = temperatureSignal.getValueAsDouble();
            inputs.mechanismAccelerationPerSecondPerSecondInMechanismUnits =
                    mechanismAccelerationSignal.getValueAsDouble();
        });
        inputs.mechanismEndUnit = mechanismEndUnit;
        inputs.motorVelocityRPS = rotorVelocitySignal.getValueAsDouble();
        inputs.mechanismRawPositionInMechanismUnits = rawMechanismPositionSignal.getValueAsDouble();
        inputs.mechanismVelocityPerSecondInMechanismUnits = mechanismVelocitySignal.getValueAsDouble();

        // No talon.getRotor...() for rotorAcceleration
        inputs.motorAccelerationRotationsPerSecondPerSecond =
                inputs.mechanismAccelerationPerSecondPerSecondInMechanismUnits * motorRotationsToMechanismUnitsRatio;
        inputs.connected = signals.isAllGood();
    }

    @Override
    public TalonFX getTalon() {
        return talon;
    }

    @Override
    public BaseMotorConfig getConfig() {
        return this.config;
    }

    @Override
    public double getMotorRotationsToMechanismUnitsRatio() {
        return motorRotationsToMechanismUnitsRatio;
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        talon.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setMechanismPositionSetpoint(double mechanismPosition) {
        setMechanismPositionSetpoint(mechanismPosition, 0);
    }

    @Override
    public void setMechanismPositionSetpoint(double mechanismPosition, int slot) {
        talon.setControl(positionVoltageControl.withPosition(mechanismPosition).withSlot(slot));
    }

    @Override
    public void setMotionMagicConfig(MotionMagicConfigs config) {
        this.config.getMotorConfiguration().MotionMagic = config;
        talon.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagicSetpoint(double mechanismPosition, int slot) {
        talon.setControl(
                motionMagicPositionControl.withPosition(mechanismPosition).withSlot(slot));
    }

    @Override
    public void setMotionMagicSetpoint(
            double mechanismPosition,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedForwards) {
        talon.setControl(dynamicMotionMagicVoltage
                .withPosition(mechanismPosition)
                .withAcceleration(acceleration)
                .withJerk(jerk)
                .withVelocity(velocity)
                .withSlot(slot)
                .withFeedForward(feedForwards));
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        TalonFXConfiguration motorConfiguration = config.getMotorConfiguration();

        if (motorConfiguration.MotorOutput.NeutralMode.value != mode.value) {
            motorConfiguration.MotorOutput.NeutralMode = mode;
            talon.setNeutralMode(motorConfiguration.MotorOutput.NeutralMode);
        }
    }

    @Override
    public void setEnableSoftLimits(boolean fwd, boolean rev) {
        TalonFXConfiguration motorConfiguration = config.getMotorConfiguration();

        if (motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable != fwd
                || motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable != rev) {
            motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
            motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
            talon.getConfigurator().apply(motorConfiguration.SoftwareLimitSwitch);
        }
    }

    @Override
    public void setEnableHardLimits(boolean fwd, boolean rev) {
        TalonFXConfiguration motorConfiguration = config.getMotorConfiguration();

        if (motorConfiguration.HardwareLimitSwitch.ForwardLimitEnable != fwd
                || motorConfiguration.HardwareLimitSwitch.ReverseLimitEnable != rev) {
            motorConfiguration.HardwareLimitSwitch.ForwardLimitEnable = fwd;
            motorConfiguration.HardwareLimitSwitch.ReverseLimitEnable = rev;
            talon.getConfigurator().apply(motorConfiguration.HardwareLimitSwitch);
        }
    }

    @Override
    public void setEnableAutosetPositionValue(boolean fwd, boolean rev) {
        TalonFXConfiguration motorConfiguration = config.getMotorConfiguration();

        if (motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable != fwd
                || motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable != rev) {
            motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = fwd;
            motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = rev;
            motorConfiguration.HardwareLimitSwitch.ForwardLimitEnable = fwd;
            motorConfiguration.HardwareLimitSwitch.ReverseLimitEnable = rev;
            talon.getConfigurator().apply(motorConfiguration.HardwareLimitSwitch);
        }
    }

    @Override
    public void setMechanismVelocityPerSecondSetpoint(double velocityPerSecond) {
        talon.setControl(velocityVoltageControl.withVelocity(velocityPerSecond).withSlot(0));
    }

    @Override
    public void setMechanismVelocityPerSecondSetpoint(double velocityPerSecond, int slot) {
        talon.setControl(velocityVoltageControl.withVelocity(velocityPerSecond).withSlot(slot));
    }

    @Override
    public void setEncoderPositionAsZero() {
        setEncoderPositionInMechanismUnits(0.0);
    }

    @Override
    public void setEncoderPositionInMechanismUnits(double mechanismPosition) {
        talon.setPosition(mechanismPosition);
    }

    @Override
    public void setVoltageOutput(double voltage) {
        talon.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void follow(CanDeviceId leaderID, MotorAlignmentValue motorAlignment) {
        talon.setControl(
                followerControl.withLeaderID(leaderID.getDeviceNumber()).withMotorAlignment(motorAlignment));
    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        talon.setControl(torqueCurrentFOC.withOutput(current));
    }

    @Override
    public void setVoltageConfig(VoltageConfigs inputConfig) {
        talon.getConfigurator().apply(inputConfig, 0.01);
    }
}
