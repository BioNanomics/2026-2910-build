package org.frc2910.robot.subsystems.base;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.frc2910.robot.util.CanDeviceId;
import org.littletonrobotics.junction.AutoLog;

/**
 * MotorIO for all motor implementations, used by all BaseSubsystems that require a motor
 */
public interface MotorIO {

    @AutoLog
    class MotorIOInputs {
        public String mechanismEndUnit = "David Units";
        public boolean connected = false;

        // Uses talon.getRotor...() that returns values dependent on only the motor
        public double motorRawPositionRotations = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAccelerationRotationsPerSecondPerSecond = 0.0;
        public double motorVoltage = 0.0;
        public double motorStatorCurrentAmps = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
        public double motorTemperatureC = 0.0;

        // Uses talon.get...() that returns values dependent on rotorToSensorRatio and sensorToMechanismRatio
        // The units that the mechanism uses will be defined in its config
        public double mechanismRawPositionInMechanismUnits = 0.0;
        public double mechanismVelocityPerSecondInMechanismUnits = 0.0;
        public double mechanismAccelerationPerSecondPerSecondInMechanismUnits = 0.0;
    }

    // General and non-movement
    void updateInputs(MotorIOInputs inputs);

    BaseMotorConfig getConfig();

    TalonFX getTalon();

    double getMotorRotationsToMechanismUnitsRatio();

    default void setNeutralMode(NeutralModeValue mode) {}

    default void setEnableSoftLimits(boolean forward, boolean reverse) {}

    default void setEnableHardLimits(boolean forward, boolean reverse) {}

    default void setEnableAutosetPositionValue(boolean forward, boolean reverse) {}

    default void setTorqueCurrentFOC(double current) {}

    // Not velocity/position specific movement:
    default void setOpenLoopDutyCycle(double dutyCycle) {}

    default void setVoltageOutput(double voltage) {}

    default void follow(CanDeviceId leaderID, MotorAlignmentValue motorAlignment) {}

    default void setVoltageConfig(VoltageConfigs config) {}

    // Servo

    default void setMechanismPositionSetpoint(double mechanismPosition) {}

    default void setMechanismPositionSetpoint(double mechanismPosition, int slot) {}

    default void setEncoderPositionAsZero() {}

    default void setEncoderPositionInMechanismUnits(double mechanismPosition) {}

    // Roller
    default void setMechanismVelocityPerSecondSetpoint(double mechanismVelocity) {}

    default void setMechanismVelocityPerSecondSetpoint(double mechanismVelocity, int slot) {}

    // MotionMagic(servo) and other
    default void setMotionMagicSetpoint(double mechanismPosition, int slot) {}

    default void setMotionMagicSetpoint(double mechanismPosition, double velocity, double acceleration, double jerk) {
        setMotionMagicSetpoint(mechanismPosition, velocity, acceleration, jerk, 0);
    }

    default void setMotionMagicSetpoint(
            double mechanismPosition, double velocity, double acceleration, double jerk, int slot) {
        setMotionMagicSetpoint(mechanismPosition, velocity, acceleration, jerk, slot, 0.0);
    }

    default void setMotionMagicSetpoint(
            double mechanismPosition,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedforward) {}

    default void setMotionMagicConfig(MotionMagicConfigs config) {}
}
