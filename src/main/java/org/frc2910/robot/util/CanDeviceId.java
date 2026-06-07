package org.frc2910.robot.util;

import com.ctre.phoenix6.CANBus;

/**
 * A class representing a CAN device ID, which consists of a numeric device number and a CAN bus name.
 * <p>This class wraps the numeric device ID together with a CTRE {@link CANBus} instance.
 * </p>
 */
public class CanDeviceId {
    private final int deviceNumber;
    private final CANBus bus;

    /**
     * Construct a CanDeviceId.
     *
     * @param deviceNumber the numeric device id on the bus
     * @param bus the name of the CAN bus (e.g. "CANivore", "rio")
     */
    public CanDeviceId(int deviceNumber, String bus) {
        this.deviceNumber = deviceNumber;
        this.bus = new CANBus(bus);
    }

    // Use the default bus name "rio".

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBusName() {
        return bus.getName();
    }

    public CANBus getBus() {
        return bus;
    }

    @SuppressWarnings("NonOverridingEquals")
    public boolean equals(CanDeviceId other) {
        return other.deviceNumber == deviceNumber && other.bus.equals(bus);
    }

    @Override
    public int hashCode() {
        int result = Integer.hashCode(deviceNumber);
        result = 31 * result + bus.hashCode();
        return result;
    }

    @Override
    public String toString() {
        return "CanDeviceId(" + deviceNumber + ", " + bus + ")";
    }
}
