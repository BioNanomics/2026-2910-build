package org.frc2910.robot.subsystems.base;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

/**
 * FusedCanCoderIO for all TalonFX fused to CanCoder implementations, used by all BaseSubsystems that require a CanCoder fused to a TalonFX.
 */
public interface FusedCanCoderIO {

    @AutoLog
    class FusedCanCoderIOInputs {
        public Angle absolutePosition;
        public AngularVelocity angularVelocity;
    }

    void updateInputs(FusedCanCoderIOInputs inputs);

    void setZeroPoint();
}
