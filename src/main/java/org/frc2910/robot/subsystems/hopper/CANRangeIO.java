package org.frc2910.robot.subsystems.hopper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import org.frc2910.robot.config.RangeConfiguration;
import org.frc2910.robot.util.DelayedBatchSignalLogger;
import org.frc2910.robot.util.StatusSignalRefresher;

public class CANRangeIO implements RangeIO {
    private final CANrange range;
    private CANrangeConfiguration canRangeConfiguration;

    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<Boolean> tripped;
    private final StatusSignal<Distance> distance;

    public CANRangeIO(RangeConfiguration config) {
        canRangeConfiguration = config.getCANrangeConfiguration();
        range = new CANrange(
                config.getCANID().getDeviceNumber(), config.getCANID().getBus());
        DelayedBatchSignalLogger.getInstance()
                .writeString("Phoenix6/CANrange-" + config.getCANID().getDeviceNumber() + "-Name", config.getName());

        range.getConfigurator().apply(canRangeConfiguration);

        signalStrength = range.getSignalStrength();
        tripped = range.getIsDetected();
        distance = range.getDistance();

        // Add all signals to the collection for easy refreshing in refreshData()
        StatusSignalRefresher.getInstance().addStatusSignals(range.getNetwork(), signalStrength, tripped, distance);
    }

    @Override
    public void updateInputs(RangeIO.RangeIOInputs inputs) {
        // Update the inputs object with the latest values
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.isTripped = tripped.getValueAsDouble() == 1;
        inputs.distance = distance.getValueAsDouble();
        inputs.connected = range.isConnected();
    }
}
