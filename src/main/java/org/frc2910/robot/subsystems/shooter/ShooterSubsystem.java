package org.frc2910.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.littletonrobotics.junction.Logger;

/**
 * Coordinates the hood and flywheel subsystems to prepare and shoot fuel.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final HoodSubsystem hood;
    private final FlywheelSubsystem flywheels;

    public ShooterSubsystem(
            RobotState robotState,
            ConditionalCallbackManager callbackManager,
            MotorIO hoodIO,
            MotorIO leaderShooterRollerIO) {
        super("Serpentine");
        hood = new HoodSubsystem(robotState, callbackManager, hoodIO);
        flywheels = new FlywheelSubsystem(robotState, callbackManager, leaderShooterRollerIO);
    }

    public enum WantedState {
        OFF,
        HOME,
        RUN,
    }

    private enum SystemState {
        OFF,
        HOMING,
        RUNNING,
    }

    private SystemState systemState = SystemState.OFF;
    public WantedState wantedState = WantedState.OFF;

    @Override
    public void periodic() {
        systemState = handleStateTransition();
        Logger.recordOutput("Serpentine/WantedState", wantedState);
        Logger.recordOutput("Serpentine/SystemState", systemState);
        applyStates();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case HOME -> SystemState.HOMING;
            case RUN -> SystemState.RUNNING;
            case OFF -> SystemState.OFF;
        };
    }

    private void applyStates() {
        switch (systemState) {
            case OFF -> {
                hood.setWantedState(HoodSubsystem.WantedState.OFF);
                flywheels.setWantedState(FlywheelSubsystem.WantedState.OFF);
            }
            case HOMING -> {
                hood.setWantedState(HoodSubsystem.WantedState.HOME);
                flywheels.setWantedState(FlywheelSubsystem.WantedState.SHOOT);
                setWantedState(WantedState.RUN);
            }
            case RUNNING -> {
                hood.setWantedState(HoodSubsystem.WantedState.HOLD_POSITION);
                flywheels.setWantedState(FlywheelSubsystem.WantedState.SHOOT);
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean isAtShotConditions() {
        return hood.isHoodAtPosition() && flywheels.isShooterAtTargetRPM();
    }

    public void setShotConditions(double desiredRPM, double desiredAngle) {
        hood.setDesiredHoodAngle(desiredAngle);
        flywheels.setDesiredRPM(desiredRPM);
    }

    public boolean isHoodHomed() {
        return hood.isHomed();
    }

    public void manualHome() {
        hood.manualHome();
    }
}
