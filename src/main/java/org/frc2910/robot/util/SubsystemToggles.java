package org.frc2910.robot.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Dashboard toggles for enabling/disabling individual subsystems. All subsystems default to enabled.
 *
 * <p>A disabled subsystem keeps reading and logging its inputs but skips its state machine and commands
 * neutral output. Values are published under {@code /SmartDashboard/Subsystem Enabled/} so they show up
 * as toggleable booleans in Shuffleboard/Elastic, and are logged by AdvantageKit for replay.
 * </p>
 */
public final class SubsystemToggles {
    private static final String tableKey = "/SmartDashboard/Subsystem Enabled";

    public static final Toggle DRIVE = new Toggle("Drive");
    public static final Toggle VISION = new Toggle("Vision");
    public static final Toggle INTAKE = new Toggle("Intake");
    public static final Toggle HOPPER = new Toggle("Hopper");
    public static final Toggle HOOD = new Toggle("Hood");
    public static final Toggle FLYWHEEL = new Toggle("Flywheel");

    private SubsystemToggles() {}

    /** Forces class initialization so all entries are published on startup. */
    public static void init() {}

    public static final class Toggle {
        private final LoggedNetworkBoolean enabled;

        private Toggle(String name) {
            enabled = new LoggedNetworkBoolean(tableKey + "/" + name, true);
        }

        public boolean isEnabled() {
            return enabled.get();
        }
    }
}
