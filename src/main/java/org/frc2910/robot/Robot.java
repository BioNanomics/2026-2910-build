// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc2910.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.frc2910.robot.autos.AutoChooser;
import org.frc2910.robot.commands.ResetSwervePose;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.config.RobotIdentity;
import org.frc2910.robot.subsystems.SuperStructure;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.StatusSignalRefresher;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot entrypoint class. Responsible for configuring subsystems, initializing
 * logging, and wiring the robot lifecycle (disabled/autonomous/teleop/test) together.
 *
 * <p>This class is intentionally thin and delegates most behavior to {@link RobotContainer} and
 * subsystem classes.
 * </p>
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final ConditionalCallbackManager callbackManager;
    private final RobotContainer robotContainer;

    private AutoChooser autoChooser;

    private boolean hasEnabled = false;
    private boolean hasAddedDebugStickyElement = false;
    private final CANBus canbus;

    /**
     * Construct the Robot instance, create subsystems and perform initial setup.
     *
     * <p>Construction should not perform long-running initialization; long-lived or IO
     * initialization is performed in the lifecycle methods.
     * </p>
     */
    public Robot() {
        Shuffleboard.getTab(Constants.operatorDashboardName); // Create Operator Dashboard first
        var debugTab = Shuffleboard.getTab(Constants.debugDashboardName);
        var debugLogEntry = debugTab.add("Debug Logging (WARN: Loop Overruns)", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .withPosition(1, 1)
                .withSize(3, 1)
                .getEntry();
        callbackManager = new ConditionalCallbackManager(() -> debugLogEntry.getBoolean(false));
        callbackManager.registerCallback(() -> {
            if (!hasAddedDebugStickyElement) {
                hasAddedDebugStickyElement = true;
                debugTab.add("Debug sticky fault (restart code+Shflbrd to clear)", false)
                        .withWidget(BuiltInWidgets.kBooleanBox)
                        .withPosition(1, 2)
                        .withSize(3, 2);
            }
        });
        robotContainer = new RobotContainer(callbackManager);
        if (RobotBase.isSimulation() && Constants.useMapleSim) {
            // In sim, position the robot on the field
            robotContainer.getDriveSubsystem().resetOdometry(new Pose2d(1, 1, new Rotation2d()));
        }

        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // Needs to match the CAN bus configured in Robot configuration.
        canbus = new CANBus("CANivore");

        callbackManager.registerOneTimeCallback(StatusSignalRefresher.getInstance()::finalizeStatusSignals);
    }

    @Override
    public void robotInit() {
        try {
            var iterativeRobotBaseClass = IterativeRobotBase.class;
            var watchdogField = iterativeRobotBaseClass.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            var watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(67);

            var commandSchedulerClass = CommandScheduler.class;
            var watchdogFieldCS = commandSchedulerClass.getDeclaredField("m_watchdog");
            watchdogFieldCS.setAccessible(true);
            var watchdogCS = (Watchdog) watchdogFieldCS.get(CommandScheduler.getInstance());
            watchdogCS.setTimeout(67);
        } catch (Exception e) {
            System.out.println(e);
        }
        DriverStation.silenceJoystickConnectionWarning(true);
        initLogging();
        StatusSignalRefresher.getInstance().finalizeStatusSignals();

        autoChooser = AutoChooser.create(robotContainer);
        var operatorDashboard = Shuffleboard.getTab(Constants.operatorDashboardName);

        operatorDashboard
                .add("Auto Program", autoChooser)
                .withSize(6, 3)
                .withPosition(12, 0)
                .withWidget(BuiltInWidgets.kComboBoxChooser);

        operatorDashboard.add(
                "Rest Pose for Auto",
                new ResetSwervePose(new InstantCommand(() -> robotContainer
                        .getDriveSubsystem()
                        .resetOdometry(autoChooser.getStartingPose().orElse(new Pose2d())))) {
                    @Override
                    public boolean runsWhenDisabled() {
                        return true;
                    }
                });

        operatorDashboard.add("Reset current limits", new InstantCommand(this::configureTeleopCurrentLimits) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });

        operatorDashboard
                .addBoolean(
                        "Is Robot Homed",
                        () -> robotContainer.getSuperStructure().isHomed())
                .withPosition(0, 0)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);

        operatorDashboard
                .addBoolean("Won Auto?", () -> robotContainer.getRobotState().wonAuto())
                .withPosition(5, 0)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }

    @Override
    public void robotPeriodic() {
        StatusSignalRefresher.getInstance().refreshStatusSignals();

        CommandScheduler.getInstance().run();

        // Log CANivore status
        if (RobotBase.isReal()) {
            var canivoreStatus = canbus.getStatus();
            SignalLogger.writeBoolean("CANivoreStatus/isOk", canivoreStatus.Status.isOK());
            SignalLogger.writeString("CANivoreStatus/Status", canivoreStatus.Status.getName());
            SignalLogger.writeDouble("CANivoreStatus/Utilization", canivoreStatus.BusUtilization);
            SignalLogger.writeInteger("CANivoreStatus/OffCount", canivoreStatus.BusOffCount);
            SignalLogger.writeInteger("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount);
            SignalLogger.writeInteger("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC);
            SignalLogger.writeInteger("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC);
        }

        if (RobotBase.isSimulation() && Constants.useMapleSim) {
            robotContainer.getSimulatedRobotState().updateSim();
        }

        callbackManager.periodic(); // Call last so we update inputs from subsystems
    }

    @Override
    public void disabledInit() {
        //        robotContainer.getVisionSubsystem().setWantedState(VisionSubsystem.WantedState.DISABLED);
        autoChooser.reset(Constants.autoChooserName);
    }

    @Override
    public void disabledPeriodic() {
        autoChooser.update();
        autonomousCommand = autoChooser.getSelectedCommand().orElse(null);
    }

    @Override
    public void autonomousInit() {
        if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
            SimulatedArena.getInstance().placeGamePiecesOnField();
        }
        if (!hasEnabled) {
            hasEnabled = true;
        }

        CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        configureTeleopCurrentLimits();
        robotContainer.getSuperStructure().setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE);
    }

    private void configureTeleopCurrentLimits() {
        robotContainer.getDriveSubsystem().setStatorCurrentLimit(120.0);
        robotContainer.getIntakeSubsystem().setCurrentLimits(100.0, 40.0);
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        DriverStation.getAlliance().ifPresent(robotContainer.getSuperStructure()::setKnownTranslationsMapForAlliance);
        if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
            SimulatedArena.getInstance().placeGamePiecesOnField();
        }
    }

    @Override
    public void teleopPeriodic() {
        pollFMSForGameSpecificMessage();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        if (RobotBase.isSimulation() && Constants.useMapleSim && !hasEnabled) {
            Logger.recordOutput(
                    "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        }
    }

    private void initLogging() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);
        Logger.recordMetadata("Robot Identity", RobotIdentity.get().name());

        switch (BuildInfo.DIRTY) {
            case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
            case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
            default -> Logger.recordMetadata("GitDirty", "Unknown");
        }

        // Set up data receivers & replay source
        if (Robot.isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }

        SignalLogger.setPath(Constants.HOOT_LOGS_PATH);
        SignalLogger.enableAutoLogging(true);

        // Start AdvantageKit logger
        Logger.start();
    }

    private void pollFMSForGameSpecificMessage() {
        if (robotContainer.getRobotState().getGameSpecificMessage().isEmpty()) {
            String gameData;
            gameData = DriverStation.getGameSpecificMessage();
            if (!gameData.isEmpty()) {
                switch (gameData.charAt(0)) {
                    case 'B' -> robotContainer.getRobotState().setGameSpecificMessage("B"); // Blue case code
                    case 'R' -> robotContainer.getRobotState().setGameSpecificMessage("R"); // Red case code
                }
            }
        }
    }
}
