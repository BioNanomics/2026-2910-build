package org.frc2910.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface RangeIO {

    @AutoLog
    class RangeIOInputs {
        public double signalStrength = 0.0;
        public boolean isTripped = false;
        public double distance = 0.0;
        public boolean connected = false;
    }

    void updateInputs(RangeIOInputs inputs);
}
