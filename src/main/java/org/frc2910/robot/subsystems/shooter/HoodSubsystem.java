package org.frc2910.robot.subsystems.shooter;

import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.servo.ServoMotorSubsystem;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.SubsystemToggles;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the shooter hood angle. homing on startup to establish position.
 */
public class HoodSubsystem extends ServoMotorSubsystem {
    private double desiredHoodAngle;
    private boolean isHomed;
    private static final double HOMING_DUTY_CYCLE = -0.1;
    private static final double HOMING_MECHANISM_VELOCITY_SETPOINT = 0.5;
    private static final double HOMING_SETTLE_TIME_REQUIRED = 0.1;
    private static final double HOMING_DEGREE_TO_RESET_TO = -0.139;

    public HoodSubsystem(RobotState robotState, ConditionalCallbackManager callbackManager, MotorIO io) {
        super(robotState, callbackManager, io);
    }

    enum WantedState {
        OFF,
        HOME,
        HOLD_POSITION
    }

    private enum SystemState {
        OFF,
        HOMING,
        HOLDING_POSITION
    }

    private SystemState systemState = SystemState.OFF;
    private WantedState wantedState = WantedState.OFF;

    @Override
    public void periodic() {
        super.periodic();
        if (!SubsystemToggles.HOOD.isEnabled()) {
            setVoltageOutput(0.0);
            return;
        }
        systemState = handleStateTransition();
        Logger.recordOutput("Hood/WantedState", wantedState);
        Logger.recordOutput("Hood/SystemState", systemState);
        Logger.recordOutput("Hood/DesiredHoodAngle", desiredHoodAngle);
        applyStates();
    }

    private SystemState handleStateTransition() {
        if (!isHomed || wantedState == WantedState.HOME) {
            return SystemState.HOMING;
        }
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case HOME -> SystemState.HOMING;
            case HOLD_POSITION -> SystemState.HOLDING_POSITION;
        };
    }

    private void applyStates() {
        switch (systemState) {
            case OFF -> setVoltageOutput(0.0);
            case HOMING -> {
                isHomed = home(
                        HOMING_DUTY_CYCLE,
                        HOMING_MECHANISM_VELOCITY_SETPOINT,
                        HOMING_SETTLE_TIME_REQUIRED,
                        HOMING_DEGREE_TO_RESET_TO);

                if (isHomed) {
                    systemState = SystemState.OFF;
                    if (wantedState == WantedState.HOME) {
                        wantedState = WantedState.OFF;
                    }
                }
            }
            case HOLDING_POSITION -> setMotionMagicSetpoint(desiredHoodAngle);
        }
    }

    boolean isHoodAtPosition() {
        boolean atPosition = ((getCurrentMechanismRawPosition()
                        <= (desiredHoodAngle + Constants.ShooterConstants.HOOD_DEGREE_TOLERANCE))
                && (getCurrentMechanismRawPosition()
                        >= (desiredHoodAngle - Constants.ShooterConstants.HOOD_DEGREE_TOLERANCE)));
        Logger.recordOutput("Hood/AtPosition", atPosition);
        return atPosition;
    }

    public void manualHome() {
        setEncoderPositionInMechanismUnits(0.0);
        isHomed = true;
    }

    public boolean isHomed() {
        return isHomed;
    }

    void setWantedState(HoodSubsystem.WantedState wantedState) {
        if (this.wantedState != WantedState.HOME && wantedState == WantedState.HOME) {
            isHomed = false;
        }
        this.wantedState = wantedState;
    }

    void setDesiredHoodAngle(double desiredHoodAngle) {
        this.desiredHoodAngle = desiredHoodAngle;
    }
}
