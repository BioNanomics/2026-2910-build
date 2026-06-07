package org.frc2910.robot.config;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import org.frc2910.robot.util.CanDeviceId;

/**
 * Configuration for a CANrange sensor.
 */
public class RangeConfiguration {
    private final String name;
    private final CanDeviceId canID;
    private final CANrangeConfiguration canRangeConfiguration;

    /**
     * Construct a RangeConfiguration.
     *
     * @param name the friendly name for this sensor
     * @param canID the CAN device id (device number and bus)
     * @param canRangeConfiguration the CTRE CANrange configuration
     */
    public RangeConfiguration(String name, CanDeviceId canID, CANrangeConfiguration canRangeConfiguration) {
        this.name = name;
        this.canID = canID;
        this.canRangeConfiguration = canRangeConfiguration;
    }

    public String getName() {
        return name;
    }

    public CanDeviceId getCANID() {
        return canID;
    }

    public CANrangeConfiguration getCANrangeConfiguration() {
        return canRangeConfiguration;
    }
}
