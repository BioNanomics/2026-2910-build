package org.frc2910.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.roller.RollerMotorSubsystem;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the hopper and feeder motors using two CANrange sensors to manage fuel flow.
 * Automatically preloads fuel toward the shooter when the hopper is full but the feeder is empty.
 */
public class HopperSubsystem extends RollerMotorSubsystem {
    private final RollerMotorSubsystem rightFrontHopperMotor;
    private final RollerMotorSubsystem rightBackHopperMotor;
    private final RollerMotorSubsystem leftBackHopperMotor;
    private final RangeIO hopperRangeIO;
    private final RangeIO feederRangeIO;
    private final Debouncer debouncer;
    private final RangeIOInputsAutoLogged hopperRangeInputs;
    private final RangeIOInputsAutoLogged feederRangeInputs;
    private boolean isHopperFull;

    public HopperSubsystem(
            RobotState robotState,
            ConditionalCallbackManager callbackManager,
            MotorIO leftFrontHopperIO,
            MotorIO rightFrontHopperIO,
            MotorIO rightBackHopperIO,
            MotorIO leftBackHopperIO,
            RangeIO hopperRange,
            RangeIO feederRange) {
        super(robotState, callbackManager, leftFrontHopperIO);
        rightFrontHopperMotor = new RollerMotorSubsystem(robotState, callbackManager, rightFrontHopperIO);
        rightBackHopperMotor = new RollerMotorSubsystem(robotState, callbackManager, rightBackHopperIO);
        leftBackHopperMotor = new RollerMotorSubsystem(robotState, callbackManager, leftBackHopperIO);
        debouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);
        this.hopperRangeIO = hopperRange;
        this.feederRangeIO = feederRange;
        this.hopperRangeInputs = new RangeIOInputsAutoLogged();
        this.feederRangeInputs = new RangeIOInputsAutoLogged();
        callbackManager.registerCallback(() -> {
            Logger.processInputs("Feeder/hopperRange", hopperRangeInputs);
            Logger.processInputs("Feeder/feederRange", feederRangeInputs);
        });
    }

    public enum WantedState {
        DEFAULT,
        SHOOT,
    }

    private enum SystemState {
        OFF,
        PRELOADING,
        SHOOTING,
    }

    private SystemState systemState = SystemState.OFF;
    private WantedState wantedState = WantedState.DEFAULT;

    @Override
    public void periodic() {
        super.periodic();
        hopperRangeIO.updateInputs(hopperRangeInputs);
        feederRangeIO.updateInputs(feederRangeInputs);
        isHopperFull = debouncer.calculate(hopperRangeInputs.isTripped);
        systemState = handleStateTransition();
        Logger.recordOutput("Feeder/WantedState", wantedState);
        Logger.recordOutput("Feeder/SystemState", systemState);
        applyStates();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case DEFAULT -> {
                if (isHopperFull && !feederRangeInputs.isTripped) {
                    yield SystemState.PRELOADING;
                }
                yield SystemState.OFF;
            }
            case SHOOT -> SystemState.SHOOTING;
        };
    }

    private void applyStates() {
        switch (systemState) {
            case OFF -> setHopperDutyCycle(0.0);
            case PRELOADING -> setHopperDutyCycle(0.07);
            case SHOOTING -> setHopperDutyCycle(1.0);
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    private void setHopperDutyCycle(double dutyCycle) {
        setOpenLoopDutyCycle(dutyCycle);
        rightFrontHopperMotor.setOpenLoopDutyCycle(dutyCycle);
        rightBackHopperMotor.setOpenLoopDutyCycle(dutyCycle);
        leftBackHopperMotor.setOpenLoopDutyCycle(dutyCycle);
    }
}
