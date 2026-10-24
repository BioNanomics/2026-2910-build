package org.frc2910.robot.subsystems.intake;

import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.roller.RollerMotorSubsystem;
import org.frc2910.robot.subsystems.base.servo.ServoMotorSubsystem;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.SubsystemToggles;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the intake deploy and rollers for fuel pickup and compression.
 * The arm deploys to max angle for intaking and retracts to compress fuel in the hopper during shooting.
 * Requires homing on startup to establish the arm's zero position.
 */
public class IntakeSubsystem extends ServoMotorSubsystem {
    private final RollerMotorSubsystem rightRoller;
    private final RollerMotorSubsystem leftRoller;

    private boolean isHomed = false;
    private static final double deployAngle = Constants.IntakeConstants.INTAKE_MAX_ANGLE;
    private static final double retractAngle = 10.0;
    private static final double homingDutyCycle = -0.07;
    private static final double homingVelocityThreshold = 0.5;
    private static final double homingSettleTime = 0.1;
    private static final double homingZero = Constants.IntakeConstants.INTAKE_MIN_ANGLE - 2.0;

    private final String logKeyWantedState;
    private final String getLogKeySystemState;

    public IntakeSubsystem(
            RobotState robotState,
            ConditionalCallbackManager callbackManager,
            MotorIO servoIO,
            MotorIO rightRollerIO,
            MotorIO leftRollerIO) {
        super(robotState, callbackManager, servoIO);
        this.rightRoller = new RollerMotorSubsystem(robotState, callbackManager, rightRollerIO);
        this.leftRoller = new RollerMotorSubsystem(robotState, callbackManager, leftRollerIO);

        this.logKeyWantedState = "Subsystem/Intake/WantedState";
        this.getLogKeySystemState = "Subsystem/Intake/SystemState";
    }

    public enum WantedState {
        DEPLOY,
        INTAKE,
        RETRACT,
        HOME,
    }

    public enum SystemState {
        DEPLOYING,
        INTAKING,
        RETRACTING,
        HOMING
    }

    private SystemState systemState = SystemState.HOMING;
    public WantedState wantedState = WantedState.DEPLOY;

    @Override
    public void periodic() {
        super.periodic();
        if (!SubsystemToggles.INTAKE.isEnabled()) {
            setVoltageOutput(0.0);
            setRollerDutyCycle(0.0);
            return;
        }
        systemState = handleStateTransition();
        Logger.recordOutput(logKeyWantedState, wantedState);
        Logger.recordOutput(getLogKeySystemState, systemState);
        applyState();
    }

    public boolean isHomed() {
        return isHomed;
    }

    public SystemState handleStateTransition() {
        if (!isHomed) {
            return SystemState.HOMING;
        }

        return switch (wantedState) {
            case DEPLOY -> SystemState.DEPLOYING;
            case INTAKE -> SystemState.INTAKING;
            case RETRACT -> SystemState.RETRACTING;
            case HOME -> SystemState.HOMING;
        };
    }

    public void applyState() {
        switch (systemState) {
            case DEPLOYING -> {
                setMotionMagicSetpoint(deployAngle);
                setRollerDutyCycle(0.0);
            }
            case RETRACTING -> {
                setMotionMagicSetpoint(retractAngle);
                setRollerDutyCycle(0.15);
            }
            case INTAKING -> {
                setMotionMagicSetpoint(deployAngle);
                setRollerDutyCycle(1.0);
            }
            case HOMING -> {
                isHomed = home(homingDutyCycle, homingVelocityThreshold, homingSettleTime, homingZero);

                if (isHomed) {
                    systemState = SystemState.DEPLOYING;
                    if (wantedState == WantedState.HOME) {
                        setWantedState(WantedState.DEPLOY);
                    }
                }
            }
        }
    }

    private void setRollerDutyCycle(double dutyCycle) {
        rightRoller.setOpenLoopDutyCycle(dutyCycle);
        leftRoller.setOpenLoopDutyCycle(dutyCycle);
    }

    public void setWantedState(WantedState wantedState) {
        if (wantedState == WantedState.HOME && this.wantedState != WantedState.HOME) {
            isHomed = false;
        }
        this.wantedState = wantedState;
    }

    public void manualHome() {
        setEncoderPositionInMechanismUnits(0.0);
        isHomed = true;
    }

    public void setCurrentLimits(double statorLimit, double supplyLimit) {
        var rightRollerConfig = rightRoller.getIO().getConfig().getMotorConfiguration();
        var leftRollerConfig = leftRoller.getIO().getConfig().getMotorConfiguration();
        rightRollerConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
        rightRollerConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;
        leftRollerConfig.CurrentLimits.StatorCurrentLimit = statorLimit;
        leftRollerConfig.CurrentLimits.SupplyCurrentLimit = supplyLimit;

        rightRoller.getIO().getTalon().getConfigurator().apply(rightRollerConfig.CurrentLimits);
        leftRoller.getIO().getTalon().getConfigurator().apply(leftRollerConfig.CurrentLimits);
    }

    public boolean isFullyRetracted() {
        return this.getCurrentMechanismRawPosition() < retractAngle + 2.0;
    }
}
