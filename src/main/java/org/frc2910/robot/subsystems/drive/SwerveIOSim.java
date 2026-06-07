package org.frc2910.robot.subsystems.drive; // Use MapleSim physics engine with custom drivetrain model

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.simulation.SimulatedRobotState;
import org.frc2910.robot.utils.simulation.MapleSimSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

/**
 * Simulation implementation of swerve drive - supports CTRE sim or MapleSim physics.
 */
public class SwerveIOSim extends SwerveIOCTRE {
    /**
     * Simulation update period (5ms = 200Hz for physics accuracy).
     */
    private static final double SIM_LOOP_PERIOD = 0.005;

    /**
     * Robot state tracker for field visualization.
     */
    private SimulatedRobotState simRobotState = null;

    /**
     * MapleSim drivetrain physics engine (null if using CTRE sim).
     */
    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    /**
     * Background thread for running physics simulation.
     */
    private Notifier simNotifier = null;

    private final SwerveModuleConstants[] moduleConstants;

    /**
     * Last simulation timestamp for delta time calculation (CTRE sim only).
     */
    private double lastSimTime;

    /**
     * Telemetry consumer: updates pose from MapleSim and feeds to odometry thread.
     */
    Consumer<SwerveDriveState> simTelemetryConsumer = swerveDriveState -> {
        // Guard against null during initialization
        if (simRobotState == null) {
            return;
        }

        // Override pose with MapleSim physics if enabled
        if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
            swerveDriveState.Pose = mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
        }

        // Update field visualization and forward to parent telemetry
        simRobotState.addFieldToRobot(swerveDriveState.Pose);
    };

    /**
     * Constructs simulation swerve drive with custom telemetry and background physics thread.
     *
     * @param robotState      shared robot state for pose tracking
     * @param simRobotState   field visualization tracker
     * @param constants       drivetrain constants (track width, wheelbase, etc.)
     * @param moduleConstants per-module constants (motor/encoder configs, coupling ratios)
     */
    @SuppressWarnings("unchecked")
    public SwerveIOSim(
            RobotState robotState,
            SimulatedRobotState simRobotState,
            SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
                    moduleConstants) {

        /// Pass REGULATED constants to parent CTRE drivetrain
        super(robotState, constants, moduleConstants);

        this.moduleConstants = moduleConstants;
        this.simRobotState = simRobotState;

        // Override telemetry consumer with sim-specific version
        registerTelemetry(simTelemetryConsumer);
        startSimThread();
    }

    /**
     * Initializes physics simulation thread (MapleSim or CTRE sim based on Constants.useMapleSim).
     */
    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (Constants.useMapleSim) {
            // Use MapleSim physics engine with REGULATED module constants
            mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                    Units.Seconds.of(SIM_LOOP_PERIOD),
                    Units.Pounds.of(Constants.DriveConstants.kRobotWeightPounds),
                    Units.Inches.of(Constants.DriveConstants.kBumperWidthInches),
                    Units.Inches.of(Constants.DriveConstants.kBumperLengthInches),
                    DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                    DCMotor.getKrakenX60(Constants.DriveConstants.kDriveMotorCount),
                    1.2,
                    getModuleLocations(),
                    getPigeon2(),
                    getModules(),
                    moduleConstants[0], // Front Left (encoder offset=0, inversions=false, PID tuned)
                    moduleConstants[1], // Front Right
                    moduleConstants[2], // Back Left
                    moduleConstants[3]); // Back Right
            simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        } else {
            // Use CTRE's built-in physics simulation
            lastSimTime = Utils.getCurrentTimeSeconds();
            simNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        }
        simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    /**
     * Resets odometry to specified pose (updates MapleSim world pose if enabled).
     */
    @Override
    public void resetOdometry(Pose2d pose) {
        if (Constants.useMapleSim && mapleSimSwerveDrivetrain != null) {
            // Update MapleSim physics world pose
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05); // Allow physics to settle
        }
        super.resetOdometry(pose);
    }

    /**
     * Updates sensor inputs and logs simulated pose for field visualization.
     */
    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        super.updateInputs(inputs);

        // Log simulated pose for AdvantageScope field visualization
        var pose = simRobotState.getLatestFieldToRobot();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", simRobotState.getLatestFieldToRobot());
        }
    }

    /**
     * Returns the MapleSim drivetrain instance (null if using CTRE sim).
     */
    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return mapleSimSwerveDrivetrain;
    }
}
