package org.frc2910.robot.subsystems.base.servo;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc2910.robot.Robot;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.MotorIOInputsAutoLogged;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.DelayedBatchSignalLogger;
import org.littletonrobotics.junction.Logger;

/**
 * Base Subsystem for subsystems that are characterized by position control and usually have a max/min position
 *
 * @apiNote YOU MUST use ServoMotorSubsystemConfig when creating the IO
 * @see ServoMotorSubsystemConfig
 */
public class ServoMotorSubsystem extends SubsystemBase {
    private final ServoMotorSubsystemConfig config;
    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs;
    private double mechanismPositionSetpoint = 0.0;
    private double zeroTimeStampSeconds = Double.NaN;
    private final boolean forwardLimitWasEnabled;
    private final boolean reverseLimitWasEnabled;
    protected final String motorName;

    private final String logKeyHomeHomingDutyCycle;
    private final String logKeyHomeVelocityThreshold;
    private final String logKeyHomeHomedMechanismPosition;
    private final String logKeyHomeHomed;
    private final String logKeySetOpenLoopDutyCycle;
    private final String logKeySetVoltageOutput;
    private final String logKeySetNeutralMode;
    private final String logKeySetMechanismPositionSetpoint;
    private final String logKeySetMotionMagicSetpointPosition;
    private final String logKeySetMotionMagicSetpointSlot;
    private final String logKeySetMotionMagicSetpointDynamicPosition;
    private final String logKeySetMotionMagicSetpointDynamicVelocity;
    private final String logKeySetMotionMagicSetpointDynamicAccel;
    private final String logKeySetMotionMagicSetpointDynamicJerk;
    private final String logKeySetMotionMagicSetpointDynamicSlot;
    private final String logKeySetMotionMagicSetpointDynamicFeedforwards;
    private final String logKeySetEncoderPosition;

    public ServoMotorSubsystem(RobotState robotState, ConditionalCallbackManager callbackManager, MotorIO io) {
        super(io.getConfig().getMotorName());
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + io.getConfig().getTalonCANID().getDeviceNumber() + "-Name",
                        io.getConfig().getMotorName());
        this.config = (ServoMotorSubsystemConfig) io.getConfig();
        this.io = io;
        this.inputs = new MotorIOInputsAutoLogged();
        this.motorName = getName();

        this.forwardLimitWasEnabled = io.getConfig().getMotorConfiguration().SoftwareLimitSwitch.ForwardSoftLimitEnable;
        this.reverseLimitWasEnabled = io.getConfig().getMotorConfiguration().SoftwareLimitSwitch.ReverseSoftLimitEnable;

        logKeyHomeHomingDutyCycle = motorName + "/API/home/homingDutyCycle";
        logKeyHomeVelocityThreshold = motorName + "/API/home/mechanismVelocityPerSecondThreshold";
        logKeyHomeHomedMechanismPosition = motorName + "/API/home/settingHomedMechanismPositionTo:";
        logKeyHomeHomed = motorName + "/API/home/homed";
        logKeySetOpenLoopDutyCycle = motorName + "/API/setOpenLoopDutyCycle/dutyCycle";
        logKeySetVoltageOutput = motorName + "/API/setVoltageOutput/voltage";
        logKeySetNeutralMode = motorName + "/API/setNeutralMode/mode";
        logKeySetMechanismPositionSetpoint = motorName + "/API/setMechanismPositionSetpoint/mechanismPosition";
        logKeySetMotionMagicSetpointPosition = motorName + "/API/setMotionMagicSetpoint/mechanismPosition";
        logKeySetMotionMagicSetpointSlot = motorName + "/API/setMotionMagicSetpoint/Slot";
        logKeySetMotionMagicSetpointDynamicPosition =
                motorName + "/API/setMotionMagicSetpointDynamic/mechanismPosition";
        logKeySetMotionMagicSetpointDynamicVelocity = motorName + "/API/setMotionMagicSetpointDynamic/Velocity";
        logKeySetMotionMagicSetpointDynamicAccel = motorName + "/API/setMotionMagicSetpointDynamic/Accel";
        logKeySetMotionMagicSetpointDynamicJerk = motorName + "/API/setMotionMagicSetpointDynamic/Jerk";
        logKeySetMotionMagicSetpointDynamicSlot = motorName + "/API/setMotionMagicSetpointDynamic/Slot";
        logKeySetMotionMagicSetpointDynamicFeedforwards = motorName + "/API/setMotionMagicSetpointDynamic/Feedforwards";
        logKeySetEncoderPosition = motorName + "/API/setEncoderPosition/position";
        callbackManager.registerCallback(() -> {
            Logger.processInputs(motorName, inputs);
        });
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    /**
     * Allows for restricting pos to mechanism max/min position at subsystem layer. Limits are also automatically applied at talon layer via softLimits
     */
    public double restrictMechanismPosition(double mechanismPosition) {
        return MathUtil.clamp(
                mechanismPosition,
                config.getMotorConfiguration().SoftwareLimitSwitch.ReverseSoftLimitThreshold,
                config.getMotorConfiguration().SoftwareLimitSwitch.ForwardSoftLimitThreshold);
    }

    /**
     * Method to home a servo mechanism. Sets mechanism position to zero once homed. Automatically disables soft-limits before homing and re-enables to original config after successfully homed.
     *
     * @param homingDutyCycle                     Duty Cycle percentage to use whilst homing
     * @param mechanismVelocityPerSecondThreshold Velocity threshold for movement to be considered stopped and therefore homed
     * @param settleTimeRequiredSeconds           Time required for velocity to be below threshold for home to count as being homed
     * @return Returns whether mechanism is homed or not
     */
    public boolean home(
            double homingDutyCycle, double mechanismVelocityPerSecondThreshold, double settleTimeRequiredSeconds) {
        return home(homingDutyCycle, mechanismVelocityPerSecondThreshold, settleTimeRequiredSeconds, 0.0);
    }

    /**
     * Method to home a servo mechanism. Automatically disables soft-limits before homing and re-enables to original config after successfully homed.
     *
     * @param homingDutyCycle                     Duty Cycle percentage to use whilst homing
     * @param mechanismVelocityPerSecondThreshold Velocity threshold for movement to be considered stopped and therefore homed
     * @param settleTimeRequiredSeconds           Time required for velocity to be below threshold for home to count as being homed
     * @param homedMechanismPosition              What the position of the mechanism is set to once mechanism is considered homed
     * @return Returns whether mechanism is homed or not
     */
    public boolean home(
            double homingDutyCycle,
            double mechanismVelocityPerSecondThreshold,
            double settleTimeRequiredSeconds,
            double homedMechanismPosition) {

        if (Robot.isSimulation()) {
            this.setEncoderPositionInMechanismUnits(homedMechanismPosition);
            return true;
        }

        io.setEnableSoftLimits(false, false);
        Logger.recordOutput(logKeyHomeHomingDutyCycle, homingDutyCycle);
        Logger.recordOutput(logKeyHomeVelocityThreshold, mechanismVelocityPerSecondThreshold);
        Logger.recordOutput(logKeyHomeHomedMechanismPosition, homedMechanismPosition);

        // Do not want to home in disabled
        if (!DriverStation.isDisabled()) {
            // If velocity is within tolerance, check time spent on homing
            if (Math.abs(inputs.mechanismVelocityPerSecondInMechanismUnits) <= mechanismVelocityPerSecondThreshold) {
                // If mechanism hasn't homed long enough, keep homing, otherwise stop
                if (Double.isNaN(zeroTimeStampSeconds)) {
                    zeroTimeStampSeconds = Timer.getFPGATimestamp();
                } else if ((Timer.getFPGATimestamp() - zeroTimeStampSeconds) >= settleTimeRequiredSeconds) {
                    zeroTimeStampSeconds = Double.NaN;
                    io.setOpenLoopDutyCycle(0.0);
                    this.setEncoderPositionInMechanismUnits(homedMechanismPosition);
                    io.setEnableSoftLimits(forwardLimitWasEnabled, reverseLimitWasEnabled);
                    Logger.recordOutput(logKeyHomeHomed, true);
                    return true;
                }
                io.setOpenLoopDutyCycle(homingDutyCycle);
                Logger.recordOutput(logKeyHomeHomed, false);
                return false;
            }
            io.setOpenLoopDutyCycle(homingDutyCycle);
            zeroTimeStampSeconds = Double.NaN;
            Logger.recordOutput(logKeyHomeHomed, false);
            return false;
        }
        Logger.recordOutput(logKeyHomeHomed, false);
        return false;
    }

    public void setMotionMagicConfig(MotionMagicConfigs config) {
        io.setMotionMagicConfig(config);
    }

    /**
     * Sets motor to a percentage of available voltage (dutyCycle is -1<=dutyCycle<=1 representing 0-100 percent neg and pos)
     **/
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

    public void setMechanismPositionSetpoint(double mechanismPosition) {
        setMechanismPositionSetpoint(mechanismPosition, 0);
    }

    public void setMechanismPositionSetpoint(double mechanismPosition, int slot) {
        mechanismPositionSetpoint = mechanismPosition;
        Logger.recordOutput(logKeySetMechanismPositionSetpoint, mechanismPosition);
        io.setMechanismPositionSetpoint(mechanismPosition, slot);
    }

    public void setMotionMagicSetpoint(double mechanismPosition) {
        setMotionMagicSetpoint(mechanismPosition, 0);
    }

    public void setMotionMagicSetpoint(double mechanismPosition, int slot) {
        mechanismPositionSetpoint = mechanismPosition;
        Logger.recordOutput(logKeySetMotionMagicSetpointPosition, mechanismPosition);
        Logger.recordOutput(logKeySetMotionMagicSetpointSlot, slot);
        io.setMotionMagicSetpoint(mechanismPosition, slot);
    }

    public void setMotionMagicSetpoint(double mechanismPosition, MotionMagicConfigs config, int slot) {
        mechanismPositionSetpoint = mechanismPosition;
        double velocity = config.MotionMagicCruiseVelocity;
        double acceleration = config.MotionMagicAcceleration;
        double jerk = config.MotionMagicJerk;

        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicPosition, mechanismPosition);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicVelocity, velocity);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicAccel, acceleration);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicJerk, jerk);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicSlot, slot);
        io.setMotionMagicSetpoint(mechanismPosition, velocity, acceleration, jerk, slot);
    }

    public void setMotionMagicSetpoint(
            double mechanismPosition, MotionMagicConfigs config, double feedfowards, int slot) {
        mechanismPositionSetpoint = mechanismPosition;
        double velocity = config.MotionMagicCruiseVelocity;
        double acceleration = config.MotionMagicAcceleration;
        double jerk = config.MotionMagicJerk;

        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicPosition, mechanismPosition);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicVelocity, velocity);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicAccel, acceleration);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicJerk, jerk);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicSlot, slot);
        Logger.recordOutput(logKeySetMotionMagicSetpointDynamicFeedforwards, feedfowards);
        io.setMotionMagicSetpoint(mechanismPosition, velocity, acceleration, jerk, slot, feedfowards);
    }

    public void setEncoderPositionAsZero() {
        Logger.recordOutput(logKeySetEncoderPosition, 0.0);
        io.setEncoderPositionAsZero();
    }

    public void setEncoderPositionInMechanismUnits(double mechanismPosition) {
        Logger.recordOutput(logKeySetEncoderPosition, mechanismPosition);
        io.setEncoderPositionInMechanismUnits(mechanismPosition);
    }

    public MotorIO getIO() {
        return io;
    }

    public MotorIOInputsAutoLogged getInputs() {
        return inputs;
    }

    public ServoMotorSubsystemConfig getConfig() {
        return config;
    }

    public double getCurrentMotorPositionRawRotations() {
        return inputs.motorRawPositionRotations;
    }

    public double getCurrentMechanismRawPosition() {
        return inputs.mechanismRawPositionInMechanismUnits;
    }

    public double getMechanismPositionSetpoint() {
        return mechanismPositionSetpoint;
    }
}
