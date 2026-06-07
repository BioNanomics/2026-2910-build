package org.frc2910.robot.subsystems.base;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc2910.robot.subsystems.base.servo.ServoWithFusedCanCoderSubsystemConfig;

/**
 * REAL CanCoder implementation for CanCoderIO, used by all BaseSubsystems that require a fused CanCoder.
 * @see FusedCanCoderIO
 */
public class FusedCanCoderIOImplementation implements FusedCanCoderIO {
    private final CANcoder canCoder;
    private final ServoWithFusedCanCoderSubsystemConfig config;

    private final StatusSignalCollection signals = new StatusSignalCollection();
    private final StatusSignal<Angle> absolutePositionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;

    public FusedCanCoderIOImplementation(ServoWithFusedCanCoderSubsystemConfig config) {
        this.config = config;
        canCoder = new CANcoder(config.getCanCoderID().getDeviceNumber());

        canCoder.getConfigurator().apply(config.getCanCoderConfig());

        absolutePositionSignal = canCoder.getAbsolutePosition();
        velocitySignal = canCoder.getVelocity();

        // Add all signals to the collection for easy refreshing in refreshData()
        signals.addSignals(absolutePositionSignal, velocitySignal);

        // Set a default update frequency for all signals
        signals.setUpdateFrequencyForAll(100.0); // 100 hz

        // Optimizes the device's bus utilization by reducing the update frequencies
        // of its status signals. This API defaults to an optimized update frequency
        // of 4 Hz to preserve log data.
        // All status signals that have not been explicitly given an update frequency
        // using BaseStatusSignal.setUpdateFrequency will be slowed down.
        canCoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FusedCanCoderIOInputs inputs) {
        // Refresh all signals to get the latest data from the TalonFX before updating the inputs
        signals.refreshAll();

        inputs.absolutePosition = absolutePositionSignal.getValue();
        inputs.angularVelocity = velocitySignal.getValue();
    }

    @Override
    public void setZeroPoint() {
        canCoder.setPosition(0.0);
    }

    public ServoWithFusedCanCoderSubsystemConfig getConfig() {
        return config;
    }

    public CANcoder getCanCoder() {
        return canCoder;
    }
}
