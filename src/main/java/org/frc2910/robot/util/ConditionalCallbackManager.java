package org.frc2910.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Manages a set of callbacks that are executed only when a provided boolean supplier
 * indicates callbacks should run. Useful for conditional debug or telemetry callbacks
 * that should be enabled/disabled at runtime.
 */
public class ConditionalCallbackManager {
    private final BooleanSupplier _shouldRunRegisteredCallbacks;
    private final List<Runnable> _callbacks;
    private final List<Runnable> _oneTimeCallbacks;

    public ConditionalCallbackManager(BooleanSupplier shouldRunRegisteredCallbacks) {
        _shouldRunRegisteredCallbacks = shouldRunRegisteredCallbacks;
        _callbacks = new ArrayList<>();
        _oneTimeCallbacks = new ArrayList<>();
    }

    public void registerCallback(Runnable func) {
        _callbacks.add(func);
    }

    public void registerOneTimeCallback(Runnable func) {
        _oneTimeCallbacks.add(func);
    }

    public void callIfEnabled(Runnable func) {
        if (_shouldRunRegisteredCallbacks.getAsBoolean()) {
            func.run();
        }
    }

    public void periodic() {
        if (_shouldRunRegisteredCallbacks.getAsBoolean()) {
            _oneTimeCallbacks.forEach(Runnable::run);
            _oneTimeCallbacks.clear();
            _callbacks.forEach(Runnable::run);
        }
    }
}
