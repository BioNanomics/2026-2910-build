package org.frc2910.robot.subsystems.base.roller;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.MotorIOInputsAutoLogged;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.DelayedBatchSignalLogger;
import org.littletonrobotics.junction.Logger;

/**
 * Base Subsystem for subsystems that are characterized by velocity/voltage control and do not have max/min position
 * @see RollerMotorSubsystemConfig
 * @apiNote YOU MUST use RollerMotorSubsystemConfig when creating the IO
 */
public class RollerMotorSubsystem extends SubsystemBase {
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs;
    private double mechanismVelocityPerSecondSetpoint = 0.0;
    protected final String motorName;

    private final String logKeySetOpenLoopDutyCycle;
    private final String logKeySetVoltageOutput;
    private final String logKeySetNeutralMode;
    private final String logKeySetMechanismVelocityPerSecondSetpoint;

    public RollerMotorSubsystem(RobotState robotState, ConditionalCallbackManager callbackManager, MotorIO io) {
        super(io.getConfig().getMotorName());
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + io.getConfig().getTalonCANID().getDeviceNumber() + "-Name",
                        io.getConfig().getMotorName());
        this.motorName = getName();

        this.io = io;
        this.inputs = new MotorIOInputsAutoLogged();

        logKeySetOpenLoopDutyCycle = motorName + "/API/setOpenLoopDutyCycle/dutyCycle";
        logKeySetVoltageOutput = motorName + "/API/setVoltageOutput/voltage";
        logKeySetNeutralMode = motorName + "/API/setNeutralMode/mode";
        logKeySetMechanismVelocityPerSecondSetpoint =
                motorName + "/API/setMechanismVelocityPerSecondSetpoint/velocityPerSecond";
        callbackManager.registerCallback(() -> Logger.processInputs(motorName, inputs));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setMotionMagicConfig(MotionMagicConfigs config) {
        io.setMotionMagicConfig(config);
    }

    /** Sets motor to a percentage of available voltage (dutyCycle is -1<=dutyCycle<=1 representing 0-100 percent neg and pos) **/
    public void setOpenLoopDutyCycle(double dutyCycle) {
        Logger.recordOutput(logKeySetOpenLoopDutyCycle, dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    public void setVoltageOutput(double voltage) {
        Logger.recordOutput(logKeySetVoltageOutput, voltage);
        io.setVoltageOutput(voltage);
    }

    public void setNeutralMode(NeutralModeValue mode) {
        Logger.recordOutput(logKeySetNeutralMode, mode);
        io.setNeutralMode(mode);
    }

    public void setMechanismVelocityPerSecondSetpoint(double velocityPerSecond) {
        setMechanismVelocityPerSecondSetpoint(velocityPerSecond, 0);
    }

    public void setMechanismVelocityPerSecondSetpoint(double velocityPerSecond, int slot) {
        mechanismVelocityPerSecondSetpoint = velocityPerSecond;
        Logger.recordOutput(logKeySetMechanismVelocityPerSecondSetpoint, velocityPerSecond);
        io.setMechanismVelocityPerSecondSetpoint(velocityPerSecond, slot);
    }

    public MotorIO getIO() {
        return io;
    }

    public MotorIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public double getCurrentMotorVoltage() {
        return inputs.motorVoltage;
    }

    public double getCurrentMotorVelocityPerSecond() {
        return inputs.motorVelocityRPS;
    }

    public double getCurrentMechanismVelocityPerSecond() {
        return inputs.mechanismVelocityPerSecondInMechanismUnits;
    }

    public double getMechanismVelocityPerSecondSetpoint() {
        return mechanismVelocityPerSecondSetpoint;
    }
}
