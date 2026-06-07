package org.frc2910.robot.config;

import edu.wpi.first.hal.HALUtil;
import org.frc2910.robot.Robot;

import java.util.Collections;
import java.util.Map;

/**
 * Enumeration identifying which physical robot or simulation environment is currently running.
 *
 * <p>Used to select the appropriate hardware configuration and subsystem behavior.
 * </p>
 */
public enum RobotIdentity {
    REBLITZ,
    SIMULATION;

    /**
     * Detect the current robot identity.
     *
     * <p>Returns SIMULATION when running in simulation mode; otherwise, determines the robot
     * based on the RoboRIO serial number, defaulting to REBLITZ if the serial is unmapped.
     * </p>
     *
     * @return the detected RobotIdentity
     */
    public static RobotIdentity get() {
        if (!Robot.isReal()) {
            return SIMULATION;
        }
        // Return RobotIdentity based on serial number
        return IDENTITY_MAP.getOrDefault(HALUtil.getSerialNumber(), DEFAULT);
    }

    private static final RobotIdentity DEFAULT = RobotIdentity.REBLITZ;
    private static final Map<String, RobotIdentity> IDENTITY_MAP = Collections.emptyMap();
}
