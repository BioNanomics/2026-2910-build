package org.frc2910.robot.subsystems.shooter;

import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.roller.RollerWithFollowersSubsystem;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.SubsystemToggles;
import org.littletonrobotics.junction.Logger;

/**
 * Controls the shooter flywheels, spinning up to a target RPM set by ShooterSubsystem.
 */
public class FlywheelSubsystem extends RollerWithFollowersSubsystem {

    public FlywheelSubsystem(
            RobotState robotState, ConditionalCallbackManager callbackManager, MotorIO leaderShooterRollerIO) {
        super(robotState, callbackManager, leaderShooterRollerIO);
    }

    enum WantedState {
        OFF,
        SHOOT
    }

    private enum SystemState {
        OFF,
        SHOOTING
    }

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;
    private double desiredRPM = 0.0;

    @Override
    public void periodic() {
        super.periodic();
        if (!SubsystemToggles.FLYWHEEL.isEnabled()) {
            setVoltageOutput(0.0);
            return;
        }
        systemState = handleStateTransitions();
        Logger.recordOutput("Flywheel/WantedState", wantedState);
        Logger.recordOutput("Flywheel/SystemState", systemState);
        Logger.recordOutput("Flywheel/DesiredRPM", desiredRPM);
        applyStates();
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case SHOOT -> SystemState.SHOOTING;
        };
    }

    private void applyStates() {
        switch (systemState) {
            case OFF -> setVoltageOutput(0.0);
            case SHOOTING -> setMechanismVelocityPerSecondSetpoint(desiredRPM / 60.0);
        }
    }

    boolean isShooterAtTargetRPM() {
        boolean isShooterAtTargetRPM = Math.abs(getCurrentMotorVelocityPerSecond() * 60 - desiredRPM)
                <= Constants.ShooterConstants.FLYWHEEL_RPM_TOLERANCE;
        Logger.recordOutput("Shooter/withinTolerance", isShooterAtTargetRPM);
        return isShooterAtTargetRPM;
    }

    void setWantedState(FlywheelSubsystem.WantedState wantedState) {
        this.wantedState = wantedState;
    }

    void setDesiredRPM(double desiredFlywheelRPM) {
        desiredRPM = desiredFlywheelRPM;
    }
}
