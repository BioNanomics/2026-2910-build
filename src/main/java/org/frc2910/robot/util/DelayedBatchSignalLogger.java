package org.frc2910.robot.util;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

/**
 * Singleton logger that batches signal logging operations and flushes them after a delay.
 *
 * <p>Useful for reducing logging overhead by deferring writes to the signal logger
 * for 10 seconds after the first write request. All queued writes are flushed together.
 * </p>
 */
public class DelayedBatchSignalLogger {
    private static DelayedBatchSignalLogger _instance;
    private final Map<String, String> _data = new HashMap<>();
    private boolean hasQueuedCommand = false;

    private DelayedBatchSignalLogger() {}

    /**
     * Get the singleton instance of the DelayedBatchSignalLogger.
     *
     * @return the logger instance
     */
    public static DelayedBatchSignalLogger getInstance() {
        if (_instance == null) {
            _instance = new DelayedBatchSignalLogger();
        }
        return _instance;
    }

    /**
     * Queue a string value to be logged after the delay period.
     *
     * <p>The value is stored and a 10-second delayed flush is queued if not already pending.
     * </p>
     *
     * @param key the signal name/key
     * @param value the string value to log
     */
    public void writeString(String key, String value) {
        if (!hasQueuedCommand) {
            hasQueuedCommand = true;
            CommandScheduler.getInstance()
                    .schedule(Commands.sequence(Commands.waitSeconds(10), Commands.runOnce(() -> {
                                _data.keySet().forEach((k) -> SignalLogger.writeString(k, _data.get(k)));
                                _data.clear();
                                hasQueuedCommand = false;
                            }))
                            .ignoringDisable(true));
        }
        _data.put(key, value);
    }
}
