// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc2910.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.config.RobotConfiguration;
import org.frc2910.robot.config.RobotIdentity;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.simulation.SimulatedRobotState;
import org.frc2910.robot.subsystems.SuperStructure;
import org.frc2910.robot.subsystems.base.MotorIOTalonFX;
import org.frc2910.robot.subsystems.base.SimMotorIOTalonFX;
import org.frc2910.robot.subsystems.drive.SwerveIOCTRE;
import org.frc2910.robot.subsystems.drive.SwerveIOSim;
import org.frc2910.robot.subsystems.drive.SwerveSubsystem;
import org.frc2910.robot.subsystems.hopper.CANRangeIO;
import org.frc2910.robot.subsystems.hopper.HopperSubsystem;
import org.frc2910.robot.subsystems.intake.IntakeSubsystem;
import org.frc2910.robot.subsystems.shooter.ShooterSubsystem;
import org.frc2910.robot.subsystems.vision.VisionIOLimelight;
import org.frc2910.robot.subsystems.vision.VisionSubsystem;
import org.frc2910.robot.util.*;

import static org.frc2910.robot.subsystems.SuperStructure.WantedState.*;

/**
 * Central container for robot subsystems, interpolations and input bindings.
 *
 * <p>Constructing this class instantiates and configures the robot subsystems and sets up
 * controller bindings for teleop operation.
 * </p>
 */
public class RobotContainer {
    private final VisionInterpolation visionInterpolation;
    private final ShooterInterpolation shooterInterpolation;
    private final PassingInterpolation passingInterpolation;

    private final ConditionalCallbackManager callbackManager;
    private final RobotState robotState;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final RobotConfiguration configuration;
    private final SimulatedRobotState simulatedRobotState = Robot.isSimulation() ? new SimulatedRobotState(this) : null;
    private final OperatorDashboard operatorDashboard = new OperatorDashboard();

    private final SwerveSubsystem swerveSubsystem;
    private final SuperStructure superStructure;

    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer(ConditionalCallbackManager callbackManager) {
        this.callbackManager = callbackManager;
        configuration = RobotConfiguration.getRobotConfiguration(RobotIdentity.get());
        robotState = new RobotState();
        SubsystemToggles.init();
        // SwerveSubsystem needs to be built first so that it can update the robot pose
        // from CTRE's pose estimator before other subsystems run. While this isn't part
        // of the CommandScheduler contract, this is the scheduler behavior in 2026.
        swerveSubsystem = buildDriveSystem();
        visionInterpolation = new VisionInterpolation();
        shooterInterpolation = new ShooterInterpolation();
        passingInterpolation = new PassingInterpolation();

        if (Robot.isSimulation()) {
            if (Constants.useMapleSim) {
                assert this.simulatedRobotState != null;
                this.simulatedRobotState.init();
            }
        }

        intakeSubsystem = buildIntakeSubsystem();
        shooterSubsystem = buildShooterSubsystem();
        hopperSubsystem = buildFeederSubsystem();

        superStructure = new SuperStructure(this);
        var cameraConfigs = configuration.getCameraConfigurations();
        var visionIOs = new VisionIOLimelight[cameraConfigs.size()];
        for (int i = 0; i < visionIOs.length; i++) {
            visionIOs[i] = new VisionIOLimelight(cameraConfigs.get(i), this, robotState);
        }
        visionSubsystem = new VisionSubsystem(robotState, visionIOs);

        configureBindings();
    }

    /**
     * Get the swerve drive subsystem instance.
     *
     * @return the drive subsystem
     */
    public SwerveSubsystem getDriveSubsystem() {
        return swerveSubsystem;
    }

    /**
     * Get the interpolation helper used for shooter tuning/lookup.
     *
     * @return the shooter interpolation instance
     */
    public ShooterInterpolation getShooterInterpolation() {
        return shooterInterpolation;
    }

    /**
     * Get the interpolation helper used for passing calculations.
     *
     * @return the passing interpolation instance
     */
    public PassingInterpolation getPassingInterpolation() {
        return passingInterpolation;
    }

    /**
     * Get the top-level super-structure that coordinates shooter/hopper/intake behavior.
     *
     * @return the SuperStructure instance
     */
    public SuperStructure getSuperStructure() {
        return superStructure;
    }

    /**
     * Get the shared RobotState used across subsystems.
     *
     * @return the RobotState instance
     */
    public RobotState getRobotState() {
        return robotState;
    }

    /**
     * Get the shooter subsystem instance.
     *
     * @return the ShooterSubsystem
     */
    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    /**
     * Get the hopper/feeder subsystem instance.
     *
     * @return the HopperSubsystem
     */
    public HopperSubsystem getFeederSubsystem() {
        return hopperSubsystem;
    }

    /**
     * Get the vision subsystem instance.
     *
     * @return the VisionSubsystem
     */
    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    /**
     * Get the intake subsystem instance.
     *
     * @return the IntakeSubsystem
     */
    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    /**
     * Get the operator dashboard helper instance.
     *
     * @return the OperatorDashboard
     */
    public OperatorDashboard getOperatorDashboard() {
        return operatorDashboard;
    }

    /**
     * Get the simulated robot state (null when not running in simulation).
     *
     * @return the SimulatedRobotState or null
     */
    public SimulatedRobotState getSimulatedRobotState() {
        return simulatedRobotState;
    }

    /**
     * Get the vision interpolation helper used for vision-based calculations.
     *
     * @return the VisionInterpolation instance
     */
    public VisionInterpolation getVisionInterpolation() {
        return visionInterpolation;
    }

    private void configureBindings() {
        // ------------- Teleop Driver Controls -------------
        // Actions that don't require superstate
        swerveSubsystem.setDefaultCommand(swerveSubsystem
                .runEnd(
                        () -> swerveSubsystem.acceptTeleopInput(
                                -controller.getLeftY(), -controller.getLeftX(), -controller.getRightX()),
                        () -> swerveSubsystem.acceptTeleopInput(0, 0, 0))
                .withName("Drive Teleop Input"));

        controller.back().onTrue(Commands.runOnce(swerveSubsystem::resetRotationBasedOnAlliance));

        // Actions with superstates
        controller
                .start()
                .onTrue(Commands.runOnce(() -> superStructure.setWantedStateCommand(HOME))
                        .ignoringDisable(true))
                .onFalse(Commands.runOnce(() -> superStructure.setWantedStateCommand(DEFAULT_STATE))
                        .ignoringDisable(true));

        controller
                .leftBumper()
                .onTrue(Commands.runOnce(() -> superStructure.setIntakeWantedState(IntakeSubsystem.WantedState.INTAKE)))
                .onFalse(Commands.runOnce(
                        () -> superStructure.setIntakeWantedState(IntakeSubsystem.WantedState.DEPLOY)));

        controller
                .leftTrigger()
                .onTrue(Commands.runOnce(
                        () -> superStructure.setIntakeWantedState(IntakeSubsystem.WantedState.RETRACT)))
                .onFalse(Commands.runOnce(
                        () -> superStructure.setIntakeWantedState(IntakeSubsystem.WantedState.DEPLOY)));

        controller
                .rightBumper()
                .onTrue(Commands.runOnce(() -> superStructure.setWantedStateCommand(SCORE_ON_MOVE)))
                .onFalse(Commands.runOnce(() -> superStructure.setWantedStateCommand(DEFAULT_STATE)));

        controller
                .rightTrigger()
                .onTrue(Commands.runOnce(() -> superStructure.setWantedStateCommand(PASS)))
                .onFalse(Commands.runOnce(() -> superStructure.setWantedStateCommand(DEFAULT_STATE)));

        controller
                .rightStick()
                .whileTrue(Commands.runOnce(() -> swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.SEEK)))
                .onFalse(Commands.runOnce(() -> swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP)));

        controller
                .x() // Lower Left Back Paddle
                .whileTrue(Commands.run(this::snapIntakeDirectionToClosestNinetyDegreesOfRightJoystick))
                .onFalse(Commands.runOnce(() -> swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP)));

        // FailSafes
        // To note, if press score, then scoreWithoutObservation and release it, superstate will go to default
        controller
                .b() // Right top back paddle
                .onTrue(Commands.runOnce(() -> superStructure.setWantedStateCommand(SCORE_WITHOUT_OBSERVATION)))
                .onFalse(Commands.runOnce(() -> superStructure.setWantedStateCommand(DEFAULT_STATE)));

        controller
                .a() // Left top back paddle
                .onTrue(Commands.runOnce(() -> superStructure.setWantedStateCommand(SCORE_KNOWN_TRANSLATION)))
                .onFalse(Commands.runOnce(() -> superStructure.setWantedStateCommand(DEFAULT_STATE)));

        controller
                .x() // Lower Left Back Paddle (this both does rotation and sets ignore to true)
                .onTrue(Commands.runOnce(() -> superStructure.setIgnoreShootingTolerances(true)))
                .onFalse(Commands.runOnce(() -> superStructure.setIgnoreShootingTolerances(false)));

        controller
                .povLeft()
                .toggleOnTrue(Commands.runOnce(
                        () -> superStructure.setScoringKnownTranslation(FieldConstants.KnownTranslations.LEFT_TRENCH)));

        controller
                .povRight()
                .toggleOnTrue(Commands.runOnce(() ->
                        superStructure.setScoringKnownTranslation(FieldConstants.KnownTranslations.RIGHT_TRENCH)));

        controller
                .povUp()
                .toggleOnTrue(Commands.runOnce(() ->
                        superStructure.setScoringKnownTranslation(FieldConstants.KnownTranslations.LEFT_UPRIGHT)));

        controller
                .povDown()
                .toggleOnTrue(Commands.runOnce(() ->
                        superStructure.setScoringKnownTranslation(FieldConstants.KnownTranslations.RIGHT_UPRIGHT)));

        controller
                .y() // Right bottom back paddle
                .onTrue(Commands.runOnce(() -> superStructure.setWantedStateCommand(PASS_MANUAL_ROT)))
                .onFalse(Commands.runOnce(() -> superStructure.setWantedStateCommand(DEFAULT_STATE)));
    }

    private SwerveSubsystem buildDriveSystem() {
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] moduleConstants =
                configuration.getModuleConstants();

        // The sim swerve subsystem detects if we are using maple sim or not internally
        if (RobotBase.isSimulation()) {
            return new SwerveSubsystem(
                    robotState,
                    callbackManager,
                    new SwerveIOSim(
                            robotState,
                            simulatedRobotState,
                            configuration.getSwerveDrivetrainConstants(),
                            configuration.getModuleConstants()),
                    moduleConstants[0].SpeedAt12Volts,
                    moduleConstants[0].SpeedAt12Volts
                            / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
        } else {
            return new SwerveSubsystem(
                    robotState,
                    callbackManager,
                    new SwerveIOCTRE(
                            robotState,
                            configuration.getSwerveDrivetrainConstants(),
                            configuration.getModuleConstants()),
                    moduleConstants[0].SpeedAt12Volts,
                    moduleConstants[0].SpeedAt12Volts
                            / Math.hypot(moduleConstants[0].LocationX, moduleConstants[0].LocationY));
        }
    }

    private ShooterSubsystem buildShooterSubsystem() {
        if (RobotBase.isSimulation()) {
            return new ShooterSubsystem(
                    robotState,
                    callbackManager,
                    new SimMotorIOTalonFX(
                            configuration.getShooterConfiguration().getHoodConfig(), callbackManager),
                    new SimMotorIOTalonFX(
                            configuration.getShooterConfiguration().getLeaderRollerConfig(), callbackManager));
        } else {
            return new ShooterSubsystem(
                    robotState,
                    callbackManager,
                    new MotorIOTalonFX(configuration.getShooterConfiguration().getHoodConfig(), callbackManager),
                    new MotorIOTalonFX(
                            configuration.getShooterConfiguration().getLeaderRollerConfig(), callbackManager));
        }
    }

    private IntakeSubsystem buildIntakeSubsystem() {
        if (RobotBase.isSimulation()) {
            return new IntakeSubsystem(
                    robotState,
                    callbackManager,
                    new SimMotorIOTalonFX(configuration.getIntakeConfiguration().getServoConfig(), callbackManager),
                    new SimMotorIOTalonFX(
                            configuration.getIntakeConfiguration().getRollerRightConfig(), callbackManager),
                    new SimMotorIOTalonFX(
                            configuration.getIntakeConfiguration().getRollerLeftConfig(), callbackManager));
        } else {
            return new IntakeSubsystem(
                    robotState,
                    callbackManager,
                    new MotorIOTalonFX(configuration.getIntakeConfiguration().getServoConfig(), callbackManager),
                    new MotorIOTalonFX(configuration.getIntakeConfiguration().getRollerRightConfig(), callbackManager),
                    new MotorIOTalonFX(configuration.getIntakeConfiguration().getRollerLeftConfig(), callbackManager));
        }
    }

    private HopperSubsystem buildFeederSubsystem() {
        if (RobotBase.isSimulation()) {
            return new HopperSubsystem(
                    robotState,
                    callbackManager,
                    new SimMotorIOTalonFX(
                            configuration.getFeederConfiguration().getLeftFrontHopperConfig(), callbackManager),
                    new SimMotorIOTalonFX(
                            configuration.getFeederConfiguration().getRightFrontHopperConfig(), callbackManager),
                    new SimMotorIOTalonFX(
                            configuration.getFeederConfiguration().getRightBackHopperConfig(), callbackManager),
                    new SimMotorIOTalonFX(
                            configuration.getFeederConfiguration().getLeftBackHopperConfig(), callbackManager),
                    new CANRangeIO(configuration.getHopperRangeConfiguration()),
                    new CANRangeIO(configuration.getFeederRangeConfiguration()));
        } else {
            return new HopperSubsystem(
                    robotState,
                    callbackManager,
                    new MotorIOTalonFX(
                            configuration.getFeederConfiguration().getLeftFrontHopperConfig(), callbackManager),
                    new MotorIOTalonFX(
                            configuration.getFeederConfiguration().getRightFrontHopperConfig(), callbackManager),
                    new MotorIOTalonFX(
                            configuration.getFeederConfiguration().getRightBackHopperConfig(), callbackManager),
                    new MotorIOTalonFX(
                            configuration.getFeederConfiguration().getLeftBackHopperConfig(), callbackManager),
                    new CANRangeIO(configuration.getHopperRangeConfiguration()),
                    new CANRangeIO(configuration.getFeederRangeConfiguration()));
        }
    }

    private void snapIntakeDirectionToClosestNinetyDegreesOfRightJoystick() {
        Rotation2d rotation = returnOppositeOfClosestNinetyDegreesOnRightJoystick();
        if (rotation != null) {
            swerveSubsystem.setTargetRotation(rotation);
        }
    }

    private Rotation2d returnOppositeOfClosestNinetyDegreesOnRightJoystick() {
        double directionThreshold = 0.8;
        if (FieldConstants.isBlueAlliance()) {
            if (controller.getRawAxis(4) >= directionThreshold) {
                return Rotation2d.kCCW_90deg;
            } else if (controller.getRawAxis(5) <= -directionThreshold) {
                return Rotation2d.k180deg;
            } else if (controller.getRawAxis(4) <= -directionThreshold) {
                return Rotation2d.kCW_90deg;
            } else if (controller.getRawAxis(5) >= directionThreshold) {
                return Rotation2d.kZero;
            }
        } else {
            if (controller.getRawAxis(4) >= directionThreshold) {
                return Rotation2d.kCW_90deg;
            } else if (controller.getRawAxis(5) <= -directionThreshold) {
                return Rotation2d.kZero;
            } else if (controller.getRawAxis(4) <= -directionThreshold) {
                return Rotation2d.kCCW_90deg;
            } else if (controller.getRawAxis(5) >= directionThreshold) {
                return Rotation2d.k180deg;
            }
        }

        return null;
    }
}
