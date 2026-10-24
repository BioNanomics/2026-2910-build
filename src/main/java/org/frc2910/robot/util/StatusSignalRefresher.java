package org.frc2910.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;

import java.util.*;

public class StatusSignalRefresher {
    private static StatusSignalRefresher _instance;

    public static StatusSignalRefresher getInstance() {
        if (_instance == null) {
            _instance = new StatusSignalRefresher();
        }
        return _instance;
    }

    private final Map<Integer, List<StatusSignal<?>>> _statusSignalMap;
    private final IdentityHashMap<StatusSignal<?>, String> _signalBusNames;
    private final Map<Integer, String> _calculatedStatusSignalsPerCycle;
    // StatusSignal.refreshAll() fails with InvalidNetwork when signals span CAN buses, so each
    // refresh pattern is split into one array per bus.
    private final Map<String, List<StatusSignal<?>[]>> _calculatedStatusSignalsArrays;
    private int _cycleCount;

    private StatusSignalRefresher() {
        _statusSignalMap = new HashMap<>();
        _signalBusNames = new IdentityHashMap<>();
        _calculatedStatusSignalsPerCycle = new HashMap<>();
        _calculatedStatusSignalsArrays = new HashMap<>();
    }

    /**
     * Adds status signals to refresh every loop.
     *
     * @param bus The CAN bus the signals' device is on
     * @param signals The signals to refresh every loop
     */
    public void addStatusSignals(CANBus bus, StatusSignal<?>... signals) {
        addStatusSignals(bus, 0, signals);
    }

    /**
     * Adds status signals to refresh, delaying by N loops between
     * refreshes. 0 is every loop, 1 is every other loop, 2 is every
     * third loop, etc.
     * @param bus The CAN bus the signals' device is on
     * @param delayLoopCount The amount of loops between refreshes
     * @param signals The signals to refresh
     */
    public void addStatusSignals(CANBus bus, int delayLoopCount, StatusSignal<?>... signals) {
        if (!_statusSignalMap.containsKey(delayLoopCount)) {
            _statusSignalMap.put(delayLoopCount, new ArrayList<>());
        }
        var statusSignalList = _statusSignalMap.get(delayLoopCount);
        Collections.addAll(statusSignalList, signals);
        for (var signal : signals) {
            _signalBusNames.put(signal, bus.getName());
        }
    }

    /**
     * Groups signals by CAN bus so each group can be refreshed with a single refreshAll() call.
     */
    private List<StatusSignal<?>[]> splitByBus(List<StatusSignal<?>> signals) {
        var byBus = new LinkedHashMap<String, List<StatusSignal<?>>>();
        for (var signal : signals) {
            byBus.computeIfAbsent(_signalBusNames.get(signal), k -> new ArrayList<>())
                    .add(signal);
        }
        var arrays = new ArrayList<StatusSignal<?>[]>();
        for (var busSignals : byBus.values()) {
            arrays.add(busSignals.toArray(new StatusSignal[0]));
        }
        return arrays;
    }

    /**
     * Finalizes all status signals.
     */
    public void finalizeStatusSignals() {
        _calculatedStatusSignalsPerCycle.clear();
        _calculatedStatusSignalsArrays.clear();
        var allStatusSignalKeys = _statusSignalMap.keySet();
        var statusSignalDelayCounts = allStatusSignalKeys.stream()
                .filter(a -> a != 0)
                .sorted()
                .toList()
                .toArray(new Integer[0]);
        if (statusSignalDelayCounts.length > 0) {
            var countedValues = statusSignalDelayCounts.clone();
            int largestNumber = countedValues[countedValues.length - 1]; // Largest value at last index to start
            // If all numbers are the same we're good -- countedValues[0] will be the same as all other values,
            // otherwise we still need to iterate
            // Only reason we don't use largestNumber is due to variable access in lambdas. Yay Java.
            while (!Arrays.stream(countedValues).allMatch(a -> a.equals(countedValues[0]))) {
                for (int i = 0; i < countedValues.length; i++) {
                    while (largestNumber > countedValues[i]) {
                        countedValues[i] += statusSignalDelayCounts[i];
                    }
                    largestNumber = countedValues[i];
                }
            }
            // Keep toRefresh around for all iterations to reduce GC pressure
            var toRefresh = new ArrayList<StatusSignal<?>>();
            for (int i = 0; i < largestNumber; i++) {
                var sb = new StringBuilder();
                for (var key : _statusSignalMap.keySet()) {
                    if (i % (key + 1) == 0) {
                        toRefresh.addAll(_statusSignalMap.get(key));
                        if (!sb.isEmpty()) {
                            sb.append(",");
                        }
                        sb.append(key);
                    }
                }
                var keyPattern = sb.toString();
                _calculatedStatusSignalsPerCycle.put(i, keyPattern);
                if (!_calculatedStatusSignalsArrays.containsKey(keyPattern)) {
                    _calculatedStatusSignalsArrays.put(keyPattern, splitByBus(toRefresh));
                }
                toRefresh.clear();
            }
        } else {
            _calculatedStatusSignalsPerCycle.put(0, "0");
            _calculatedStatusSignalsArrays.put("0", splitByBus(_statusSignalMap.get(0)));
        }
    }

    /**
     * Refreshes all relevant status signals. Must be called exactly once
     * per robot loop, preferably in robotPeriodic() before all other
     * periodics are run.
     */
    public void refreshStatusSignals() {
        var calculatedCount = _calculatedStatusSignalsPerCycle.size();
        List<StatusSignal<?>[]> toRefresh;
        if (calculatedCount <= 1) {
            toRefresh = _calculatedStatusSignalsArrays.get("0");
        } else {
            var statusSignalArrayKey = _calculatedStatusSignalsPerCycle.get(_cycleCount % (calculatedCount - 1));
            toRefresh = _calculatedStatusSignalsArrays.get(statusSignalArrayKey);
        }
        for (var busSignals : toRefresh) {
            StatusSignal.refreshAll(busSignals);
        }
        _cycleCount++;
    }
}
