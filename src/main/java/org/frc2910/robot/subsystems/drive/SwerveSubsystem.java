package org.frc2910.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc2910.robot.Robot;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.Constants;
import org.frc2910.robot.config.Constants.SystemCharacterizationConstants;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.SubsystemToggles;
import org.frc2910.robot.utils.simulation.MapleSimSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.stream.IntStream;

/**
 * Swerve drive subsystem with state machine control.
 */
public class SwerveSubsystem extends SubsystemBase {
    // ---- Motion Control Constants ----

    /// Position tolerance for drive-to-point commands (1.0 inch).
    private double translationErrorMarginForDriveToPoint;

    public double translationScalar = 0.25;

    /// Static friction compensation for drive-to-point (2% of max velocity).
    public static final double DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT = 0.02;

    /// Maximum velocity output for drive-to-point (default 10 ft/s).
    public double maxVelocityOutputForDriveToPoint = Units.feetToMeters(10.0);

    /// Zero chassis speeds constant (reused to avoid allocations).
    protected static final ChassisSpeeds CHASSIS_SPEEDS_ZERO = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final RobotState robotState;

    /// Joystick deadband to eliminate drift (10% threshold).
    protected static final double DEADBAND = 0.1;

    /**
     * Skew compensation scalar for rotational drift correction during translation.
     *
     * <p>Multiplied by current omega to compute heading offset. Typical: -0.02 to -0.05.
     * </p>
     */
    protected static final double SKEW_COMPENSATION_SCALAR = -0.03;

    // ---- Controllers ----

    /// Choreo path following controllers (not yet implemented)
    //    private final PIDController choreoXController = new PIDController(7, 0, 0);
    //    private final PIDController choreoYController = new PIDController(7, 0, 0);
    //    private final PIDController choreoThetaController = new PIDController(7, 0, 0);

    /// PID controller for autonomous drive-to-point (kP=3.0, kD=0.1).
    private final PIDController autoDriveToPointController = new PIDController(3.0, 0, 0.1);

    /// PID controller for teleop drive-to-point (kP=3.6, kD=0.1).
    private final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

    /// Field-centric driving with target angle lock (velocity control).
    private final SwerveRequest.FieldCentricFacingAngle driveAtAngle =
            new SwerveRequest.FieldCentricFacingAngle().withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    ///  Puts wheels into an X-shape to resist movement
    private final SwerveRequest.SwerveDriveBrake swerveDriveBrake = new SwerveRequest.SwerveDriveBrake();

    // ---- State Machine ----

    /// Desired states that can be requested externally.
    public enum WantedState {
        SYS_ID_TRANSLATION,
        SYS_ID_ROTATION,
        SYS_ID_STEER,
        TELEOP,
        SEEK,
        FOLLOW_PATH,
        ROTATION_LOCK,
        ROTATION_LOCK_SLOW_TRANSLATION,
        ANTI_DEFENSE_WHEEL_LOCK,
        DRIVE_TO_POINT,
        TRACK_TARGET,
        SET_MODULE_POSITIONS,
        SET_STATOR_CURRENT_LIMIT,
        IDLE
    }

    /// Internal states representing actual subsystem behavior.
    private enum SystemState {
        SYS_ID,
        TELEOP_DRIVING,
        SEEKING,
        PATH_FOLLOWING,
        ROTATION_LOCKING,
        ROTATION_LOCKING_SLOW_TRANSLATION,
        ANTI_DEFENSE_WHEEL_LOCKING,
        DRIVING_TO_POINT,
        TRACKING_TARGET,
        SETTING_MODULE_POSITIONS,
        SETTING_STATOR_CURRENT_LIMIT,
        IDLING
    }

    private WantedState wantedState = WantedState.TELEOP;
    private SystemState systemState = SystemState.TELEOP_DRIVING;

    /// Last cycle timestamp for delta time calculations (seconds).
    private double lastTimestamp;

    /// Target rotation for rotation lock mode.
    private Rotation2d desiredRotationForRotationLockState;

    ///  Commented out Choreo as it's not implemented yet
    //    private Trajectory<SwerveSample> desiredChoreoTrajectory;
    //    private final Timer choreoTimer = new Timer();
    //    private Optional<SwerveSample> choreoSampleToBeApplied;

    /// Target pose for drive-to-point mode.
    private Pose2d desiredPoseForDriveToPoint = new Pose2d();

    // ---- Hardware Abstraction ----

    /// Hardware interface providing sensor data and control (CTRE, sim, or replay).
    private final SwerveIO io;

    /// Main drive inputs from odometry thread (pose, speeds, gyro data).
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    /// Per-module motor telemetry (current, voltage, temperature).
    private final ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged[] moduleInputs =
            new ModuleIOInputsAutoLogged[] {frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs};

    /// Maximum translational velocity (m/s).
    private final double maxVelocity;

    /// Maximum angular velocity (rad/s).
    private final double maxAngularVelocity;

    // ---- Teleop Input State ----

    /// Cached X-axis joystick input (forward/backward), range [-1.0, 1.0].
    private double controllerX = 0.0;

    /// Cached Y-axis joystick input (strafe left/right), range [-1.0, 1.0].
    private double controllerY = 0.0;

    /// Cached rotation joystick input (turn CW/CCW), range [-1.0, 1.0].
    private double controllerOmega = 0.0;

    /// Target yaw angle for continuous tracking mode (field-relative).
    private final Rotation2d targetYaw = Rotation2d.kZero;

    /// Previous target yaw for calculating feedforward velocity.
    private Rotation2d lastTargetYaw = Rotation2d.kZero;

    /// Filtered feedforward rate for smooth tracking (rad/s).
    private double filteredFeedforward = 0.0;

    /**
     * Pre-allocated continuous target request where driver only controls translation (reused to avoid allocations).
     *
     * <p>Combines driver joystick translation (vx, vy) with PID + feedforward rotation control.
     * </p>
     */
    private final SwerveRequest.FieldCentricFacingAngle continuousTracking = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withHeadingPID(3.0, 0.0, 0.15) // Higher P for faster response, higher D for damping
            .withDeadband(0.02);

    /// Pre-allocated teleop request for open-loop field-oriented driving (reused to avoid allocations).
    private final SwerveRequest.ApplyFieldSpeeds teleopRequest = new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    /// Pre-allocated idle request with zero speeds (reused to avoid allocations).
    private final SwerveRequest.ApplyFieldSpeeds idleRequest = new SwerveRequest.ApplyFieldSpeeds()
            .withSpeeds(CHASSIS_SPEEDS_ZERO)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    // ---- Pose Estimation ----

    /// Last recorded module positions for odometry integration.
    private final SwerveModulePosition[] lastModulePositions;

    /// Swerve drive kinematics for converting chassis speeds to module states.
    private final SwerveDriveKinematics kinematics;

    /// Teleop velocity scaling coefficient (0.0-2.0, default 1.0 for full speed).
    private final double teleopVelocityCoefficient = 1.0;

    /// Rotation velocity scaling coefficient (0.0-1.0, default 1.0 for full speed).
    private final double rotationVelocityCoefficient = 1.0;

    /// Maximum angular velocity override for drive-to-point (NaN = no limit).
    private final double maximumAngularVelocityForDriveToPoint = 5.0;

    private static final Translation2d CENTER_OF_ROTATION_OFFSET = new Translation2d(-0.09, 0.0);
    // TODO verify rotation offset:
    private static final Translation2d SEEKING_ROTATION_OFFSET = new Translation2d(-0.20, 0.0);

    Timer trajectoryTimer = new Timer();

    PathPlannerTrajectory trajectoryToFollow;

    private final PIDController trajectoryXController = new PIDController(0.0, 0, 0);
    private final PIDController trajectoryYController = new PIDController(0.0, 0, 0);
    private final PIDController trajectoryThetaController = new PIDController(0.0, 0, 0.0);

    /// Cached chassis speeds for rotation lock mode.
    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private ChassisSpeeds teleopChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    private Rotation2d[] desiredModulePositions;

    private double statorCurrentLimit = 0.0;

    private final SysIdRoutine translationSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    SystemCharacterizationConstants.DEFAULT_TRANSLATION_RAMP_RATE,
                    SystemCharacterizationConstants.DEFAULT_TRANSLATION_STEP_VOLTAGE,
                    SystemCharacterizationConstants.DEFAULT_TIMEOUT,
                    this::sysIdStateConsumer),
            new SysIdRoutine.Mechanism(this::voltageConsumer, null, this));
    private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    SystemCharacterizationConstants.DEFAULT_STEER_RAMP_RATE,
                    SystemCharacterizationConstants.DEFAULT_STEER_STEP_VOLTAGE,
                    SystemCharacterizationConstants.DEFAULT_TIMEOUT,
                    this::sysIdStateConsumer),
            new SysIdRoutine.Mechanism(this::voltageConsumer, null, this));
    private final SysIdRoutine rotationSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    SystemCharacterizationConstants.DEFAULT_ROTATION_RAMP_RATE,
                    SystemCharacterizationConstants.DEFAULT_ROTATION_STEP_VOLTAGE,
                    SystemCharacterizationConstants.DEFAULT_TIMEOUT,
                    this::sysIdStateConsumer),
            new SysIdRoutine.Mechanism(this::voltageConsumer, null, this));

    /**
     * Constructs a swerve drive subsystem with the specified hardware interface and velocity limits.
     *
     * @param robotState         shared robot state for pose tracking
     * @param io                 hardware interface (CTRE, sim, or replay)
     * @param maxVelocity        maximum translational velocity (m/s)
     * @param maxAngularVelocity maximum angular velocity (rad/s)
     */
    public SwerveSubsystem(
            RobotState robotState,
            ConditionalCallbackManager callbackManager,
            SwerveIO io,
            double maxVelocity,
            double maxAngularVelocity) {
        this.robotState = robotState;
        this.io = io;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;

        // io.registerTelemetryFunction(swerveInputs);
        driveAtAngle.HeadingController = new PhoenixPIDController(5.0, 0, 0.15);

        driveAtAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        /// This is for Choreo, and is thus commented out as it's not implemented yet
        //        choreoThetaController.enableContinuousInput(-Math.PI, Math.PI);

        kinematics = io.getSwerveKinematics();
        int moduleCount = kinematics.toSwerveModuleStates(new ChassisSpeeds()).length;

        // Pose estimator init
        lastModulePositions = new SwerveModulePosition[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            lastModulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d());
        }

        // Initialize timestamp
        lastTimestamp = Timer.getTimestamp();

        trajectoryThetaController.enableContinuousInput(-Math.PI, Math.PI);

        callbackManager.registerCallback(() -> {
            Logger.processInputs("DriveInputs", inputs);
            for (int i = 0; i < moduleInputs.length; i++) {
                Logger.processInputs("DriveModule" + i + "Inputs", moduleInputs[i]);
            }
        });
    }

    @Override
    public void periodic() {
        /// Update sensor inputs from hardware (pose, speeds, gyro data)
        io.updateInputs(inputs);
        io.updateModuleInputs(moduleInputs);
        /// Log module-specific telemetry (encoder angles, velocities, currents)
        io.logModules(inputs);

        if (!SubsystemToggles.DRIVE.isEnabled()) {
            io.setSwerveState(idleRequest);
            lastTimestamp = Timer.getFPGATimestamp();
            return;
        }

        /// Determine current system state based on desired state
        systemState = handleStateTransition();

        Logger.recordOutput("Drive/SystemState", systemState);
        Logger.recordOutput("Drive/DesiredState", wantedState);

        /// Execute control logic for current state
        applyStates();

        /// Update timestamp for next cycle's delta time calculations
        lastTimestamp = Timer.getFPGATimestamp();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case SYS_ID_TRANSLATION, SYS_ID_ROTATION, SYS_ID_STEER -> SystemState.SYS_ID;
            case TELEOP -> SystemState.TELEOP_DRIVING;
            case SEEK -> SystemState.SEEKING;
            case FOLLOW_PATH -> {
                if (systemState != SystemState.PATH_FOLLOWING) {
                    trajectoryTimer.restart();
                    Logger.recordOutput("PathPlanner/Inital pose", trajectoryToFollow.sample(0.0).pose);
                }

                boolean hasFinishedWithPose = Math.abs(getRobotDistanceFromPathPlannerEndpoint(trajectoryToFollow))
                        <= Constants.TOLERANCE_FOR_ENDING_PATH_METERS;
                boolean hasFinishedWithTime = trajectoryTimer.get() >= trajectoryToFollow.getTotalTimeSeconds();

                Logger.recordOutput("PathPlanner/hasFinishedWithPose", hasFinishedWithPose);
                Logger.recordOutput("PathPlanner/hasFinsihedWithTime", hasFinishedWithTime);
                Logger.recordOutput("PathPlanner/isPathFinished", hasFinishedWithPose || hasFinishedWithTime);

                if (hasFinishedWithPose || hasFinishedWithTime) {
                    wantedState = WantedState.IDLE;
                    yield SystemState.IDLING;
                }
                yield SystemState.PATH_FOLLOWING;
            }
            case ROTATION_LOCK_SLOW_TRANSLATION -> SystemState.ROTATION_LOCKING_SLOW_TRANSLATION;
            case ROTATION_LOCK -> SystemState.ROTATION_LOCKING;
            case ANTI_DEFENSE_WHEEL_LOCK -> SystemState.ANTI_DEFENSE_WHEEL_LOCKING;
            case DRIVE_TO_POINT -> SystemState.DRIVING_TO_POINT;
            case TRACK_TARGET -> SystemState.TRACKING_TARGET;
            case SET_MODULE_POSITIONS -> {
                if (areModulesAtPosition()) {
                    setWantedState(WantedState.IDLE);
                    yield SystemState.IDLING;
                }

                yield SystemState.SETTING_MODULE_POSITIONS;
            }
            case SET_STATOR_CURRENT_LIMIT -> SystemState.SETTING_STATOR_CURRENT_LIMIT;
            default -> SystemState.IDLING;
        };
    }

    /// Executes control logic for current system state
    private void applyStates() {
        ChassisSpeeds speeds;
        switch (systemState) {
            case SYS_ID:
                /// System identification mode - no control output (SysId commands sent externally)
                break;
            case TELEOP_DRIVING:
                /// Process driver joystick inputs --> field-oriented chassis speeds
                speeds = getChassisSpeedsFromJoysticks(
                        inputs,
                        controllerX,
                        controllerY,
                        controllerOmega,
                        teleopVelocityCoefficient,
                        rotationVelocityCoefficient);
                setTeleopChassisSpeed(speeds);
                /// Send commands to swerve modules (open-loop voltage control)
                io.setSwerveState(teleopRequest.withSpeeds(speeds).withCenterOfRotation(CENTER_OF_ROTATION_OFFSET));
                break;
            case SEEKING:
                io.setSwerveState(seekingWheelchair());
                break;
            case PATH_FOLLOWING:
                io.setSwerveState(pathFollowing());
                break;
            case ROTATION_LOCKING:
                /// Hold target rotation while driver controls translation
                speeds = getChassisSpeedsFromJoysticks(
                        inputs,
                        controllerX,
                        controllerY,
                        controllerOmega,
                        teleopVelocityCoefficient,
                        rotationVelocityCoefficient);
                io.setSwerveState(driveAtAngle
                        .withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withTargetDirection(desiredRotationForRotationLockState));
                break;
            case ROTATION_LOCKING_SLOW_TRANSLATION:
                /// Hold target rotation while driver controls translation
                speeds = getChassisSpeedsFromJoysticks(
                        inputs,
                        controllerX,
                        controllerY,
                        controllerOmega,
                        translationScalar,
                        rotationVelocityCoefficient);
                io.setSwerveState(driveAtAngle
                        .withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withTargetDirection(desiredRotationForRotationLockState));
                break;
            case ANTI_DEFENSE_WHEEL_LOCKING:
                io.setSwerveState(swerveDriveBrake);
                break;
            case DRIVING_TO_POINT:
                /// Calculate distance and direction to target point
                var translationToDesiredPoint =
                        desiredPoseForDriveToPoint.getTranslation().minus(inputs.Pose.getTranslation());
                var linearDistance = translationToDesiredPoint.getNorm();

                /// Add static friction compensation when close to target
                var frictionConstant = 0.0;
                if (linearDistance >= Units.inchesToMeters(0.5)) {
                    frictionConstant = DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT * maxVelocity;
                }

                /// Calculate velocity output using PID (auto vs teleop gains)
                var directionOfTravel = translationToDesiredPoint.getAngle();
                var velocityOutput = 0.0;
                if (DriverStation.isAutonomous()) {
                    velocityOutput = Math.min(
                            Math.abs(autoDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                            maxVelocityOutputForDriveToPoint);
                } else {
                    velocityOutput = Math.min(
                            Math.abs(teleopDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                            maxVelocityOutputForDriveToPoint);
                }

                /// Decompose velocity into X/Y components
                var xComponent = velocityOutput * directionOfTravel.getCos();
                var yComponent = velocityOutput * directionOfTravel.getSin();

                /// Log drive-to-point diagnostics
                Logger.recordOutput("Drive/DriveToPoint/xVelocitySetpoint", xComponent);
                Logger.recordOutput("Drive/DriveToPoint/yVelocitySetpoint", yComponent);
                Logger.recordOutput("Drive/DriveToPoint/velocityOutput", velocityOutput);
                Logger.recordOutput("Drive/DriveToPoint/linearDistance", linearDistance);
                Logger.recordOutput("Drive/DriveToPoint/directionOfTravel", directionOfTravel);
                Logger.recordOutput("Drive/DriveToPoint/desiredPoint", desiredPoseForDriveToPoint);

                /// Apply translation + rotation control (with optional angular rate limit)
                if (Double.isNaN(maximumAngularVelocityForDriveToPoint)) {
                    io.setSwerveState(driveAtAngle
                            .withVelocityX(xComponent)
                            .withVelocityY(yComponent)
                            .withTargetDirection(desiredPoseForDriveToPoint.getRotation()));
                } else {
                    io.setSwerveState(driveAtAngle
                            .withVelocityX(xComponent)
                            .withVelocityY(yComponent)
                            .withTargetDirection(desiredPoseForDriveToPoint.getRotation())
                            .withMaxAbsRotationalRate(maximumAngularVelocityForDriveToPoint));
                }
                break;
            case TRACKING_TARGET:
                /// Continuous Tracking Mode - Auto-aim while driver controls translation
                ///
                /// Control Pipeline:
                /// 1. Process joystick inputs --> field-oriented translation
                /// 2. Calculate feedforward based on target yaw velocity
                /// 3. Send combined translation + auto-rotation to hardware

                /// Step 1: Process driver joystick inputs --> field-oriented chassis speeds
                speeds = getChassisSpeedsFromJoysticks(
                        inputs, controllerX, controllerY, 0.0, teleopVelocityCoefficient, rotationVelocityCoefficient);

                /// Step 2: Calculate feedforward for smooth target tracking
                /// Feedforward anticipates how fast the target is moving: angle_ff = delta theta /
                // delta t
                /// Benefits: reduced lag, smoother motion, better disturbance rejection

                /// Calculate angular change, wrapping at +- pi
                var deltaTarget = targetYaw.minus(lastTargetYaw);

                /// Prevent division by zero with clamped delta time
                var dt = MathUtil.clamp(Timer.getFPGATimestamp() - lastTimestamp, 0.01, 0.1);

                /// Convert to angular velocity: rad/cycle --> rad/s
                var rawFeedforwardRate = deltaTarget.div(dt).getRadians();

                /// Clamp raw feedforward to +-2.0 rad/s to prevent violent reactions
                rawFeedforwardRate = MathUtil.clamp(rawFeedforwardRate, -2.0, 2.0);

                /// Apply exponential moving average filter (alpha=0.6 for fast response with moderate smoothing)
                double alpha = 0.6;
                filteredFeedforward = alpha * rawFeedforwardRate + (1.0 - alpha) * filteredFeedforward;

                /// Update state for next cycle's feedforward calculation
                lastTargetYaw = targetYaw;

                Logger.recordOutput("Drive/TargetYaw", targetYaw);
                Logger.recordOutput("Drive/RawFeedforwardRate degrees/s", Math.toDegrees(rawFeedforwardRate));
                Logger.recordOutput("Drive/FilteredFeedforwardRate degrees/s", Math.toDegrees(filteredFeedforward));

                Logger.recordOutput("Debug/RobotPose", inputs.Pose);
                Logger.recordOutput("Debug/TargetYaw_deg", targetYaw.getDegrees());
                Logger.recordOutput(
                        "Debug/CurrentYaw_deg", inputs.Pose.getRotation().getDegrees());
                Logger.recordOutput("Debug/DeltaTarget_deg", deltaTarget.getDegrees());
                Logger.recordOutput("Debug/RawFeedforwardRate_degPerSec", Math.toDegrees(rawFeedforwardRate));
                Logger.recordOutput("Debug/FilteredFeedforwardRate_degPerSec", Math.toDegrees(filteredFeedforward));
                Logger.recordOutput(
                        "Debug/TranslationVelocity", new double[] {speeds.vxMetersPerSecond, speeds.vyMetersPerSecond});
                Logger.recordOutput("Debug/YawVelocity_radPerSec", inputs.yawVelocityRadPerSec);

                /// Step 3: Send combined translation + PID rotation + feedforward to hardware
                io.setSwerveState(continuousTracking
                        .withTargetDirection(targetYaw)
                        .withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withTargetRateFeedforward(filteredFeedforward));
                break;
            case SETTING_MODULE_POSITIONS:
                io.setSwerveState(new SetModulePositionsRequest().withModulePositions(desiredModulePositions));
                break;
            case SETTING_STATOR_CURRENT_LIMIT:
                io.setStatorCurrentLimit(statorCurrentLimit);
                setWantedState(WantedState.TELEOP);
                break;
            case IDLING:
            default:
                /// Idle state - stop all motion (reuse pre-allocated request)
                io.setSwerveState(idleRequest);
                break;
        }
    }

    /**
     * Get the current {@link WantedState}.
     *
     * @return A {@link WantedState}.
     */
    public WantedState getWantedState() {
        return wantedState;
    }

    /**
     * Sets the desired subsystem state.
     *
     * <p>Available states: TELEOP, SYS_ID, CHOREO_PATH, ROTATION_LOCK,
     * DRIVE_TO_POINT, TRACK_TARGET, IDLE
     * </p>
     *
     * @param state the desired subsystem state
     */
    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    /**
     * Accepts teleop input from the driver controller.
     *
     * <p>Call continuously during teleop to update the robot's commanded motion.
     * Automatically switches to TELEOP state unless TRACK_TARGET is active.
     * </p>
     *
     * @param controllerX     forward/backward input [-1.0, 1.0] (left stick Y)
     * @param controllerY     strafe left/right input [-1.0, 1.0] (left stick X)
     * @param controllerOmega rotation input [-1.0, 1.0] (right stick X)
     */
    public void acceptTeleopInput(double controllerX, double controllerY, double controllerOmega) {
        if (DriverStation.isTeleopEnabled()) {
            if (wantedState != WantedState.TRACK_TARGET
                    && wantedState != WantedState.ROTATION_LOCK
                    && wantedState != WantedState.ROTATION_LOCK_SLOW_TRANSLATION
                    && wantedState != WantedState.SEEK
                    && wantedState != WantedState.ANTI_DEFENSE_WHEEL_LOCK) {
                setWantedState(WantedState.TELEOP);
            }
            this.controllerX = controllerX;
            this.controllerY = controllerY;
            this.controllerOmega = controllerOmega;
        }
    }

    /**
     * Creates field-relative ChassisSpeeds from joystick inputs.
     *
     * <p>Applies deadband, exponential scaling, alliance flipping, and skew compensation.
     * </p>
     *
     * @param inputs                      current robot state inputs
     * @param controllerX                 forward/backward joystick input [-1.0, 1.0]
     * @param controllerY                 strafe left/right joystick input [-1.0, 1.0]
     * @param controllerOmega             rotation joystick input [-1.0, 1.0]
     * @param teleopVelocityCoefficient   scaling factor for translational velocity
     * @param rotationVelocityCoefficient scaling factor for rotational velocity
     */
    protected ChassisSpeeds getChassisSpeedsFromJoysticks(
            SwerveIO.SwerveIOInputs inputs,
            double controllerX,
            double controllerY,
            double controllerOmega,
            double teleopVelocityCoefficient,
            double rotationVelocityCoefficient) {

        /// Step 1: Apply deadband to eliminate stick drift
        double xMagnitude = MathUtil.applyDeadband(controllerX, DEADBAND);
        double yMagnitude = MathUtil.applyDeadband(controllerY, DEADBAND);
        double angularMagnitude = MathUtil.applyDeadband(controllerOmega, DEADBAND);

        if (xMagnitude == 0.0 && yMagnitude == 0.0 && angularMagnitude == 0.0) {
            /// No input - return zero speeds immediately
            return CHASSIS_SPEEDS_ZERO;
        }

        /// Step 2: Square inputs while preserving sign for exponential sensitivity
        /// Provides fine control at low speeds (0.1^2 = 0.01) while maintaining full power (1.0^2 = 1.0)
        xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        /// Step 3: Scale to max velocities and flip for alliance
        /// Blue alliance: forward is positive X | Red alliance: controls flipped
        final double allianceMultiplier = FieldConstants.isBlueAlliance() ? 1.0 : -1.0;
        final double xVelocity = xMagnitude * maxVelocity * allianceMultiplier * teleopVelocityCoefficient;
        final double yVelocity = yMagnitude * maxVelocity * allianceMultiplier * teleopVelocityCoefficient;
        final double angularVelocity = angularMagnitude * maxAngularVelocity * rotationVelocityCoefficient;

        /// Step 4: Apply skew compensation to correct gyroscopic drift during rotation
        /// Offset = omega x scalar (typically -0.03)
        Rotation2d skewCompensationFactor = Robot.isReal()
                ? Rotation2d.fromRadians(inputs.yawVelocityRadPerSec * SKEW_COMPENSATION_SCALAR)
                : Rotation2d.kZero;

        /// Transform: Field --> Robot --> Field (with compensated heading)

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        /// DON'T Negate angular velocity for CCW+ convention
                        /// already applied Negative at Joystick Input to make Input match CCW+
                        // Convention
                        new ChassisSpeeds(xVelocity, yVelocity, angularVelocity), inputs.Pose.getRotation()),
                /// Apply skew compensation
                inputs.Pose.getRotation().plus(skewCompensationFactor));
    }

    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
    }

    /// Sets target pose for drive-to-point with custom max translational velocity
    public void setDesiredPoseForDriveToPoint(
            Pose2d pose, double maxVelocityOutputForDriveToPoint, double toleranceInMeters) {
        this.desiredPoseForDriveToPoint = pose;
        Logger.recordOutput("DriveToPoint/Point", desiredPoseForDriveToPoint);
        translationErrorMarginForDriveToPoint = toleranceInMeters;
        Logger.recordOutput("DriveToPoint/TranslationalTolerance", translationErrorMarginForDriveToPoint);
        this.wantedState = WantedState.DRIVE_TO_POINT;
        this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
    }

    public void setDesiredPoseForDriveToPoint(Pose2d pose, double maxVelocityOutputForDriveToPoint) {
        this.desiredPoseForDriveToPoint = pose;
        Logger.recordOutput("DriveToPoint/Point", desiredPoseForDriveToPoint);
        translationErrorMarginForDriveToPoint = Units.inchesToMeters(6.0);
        Logger.recordOutput("DriveToPoint/TranslationalTolerance", translationErrorMarginForDriveToPoint);
        this.wantedState = WantedState.DRIVE_TO_POINT;
        this.maxVelocityOutputForDriveToPoint = maxVelocityOutputForDriveToPoint;
    }

    /// Resets only robot translation (preserves heading)
    public void resetTranslation(Translation2d translation2d) {
        io.resetRobotTranslation(translation2d);
    }

    /// Resets rotation based on alliance (0degrees blue, 180degrees red)
    public void resetRotationBasedOnAlliance() {
        io.resetRotation();
    }

    /// Sets target rotation for rotation lock mode
    public void setTargetRotation(Rotation2d desiredRotation) {
        this.desiredRotationForRotationLockState = desiredRotation;
        setWantedState(WantedState.ROTATION_LOCK);
    }

    public void setTargetRotationWithLimitedTranslation(Rotation2d desiredRotation, double translationScalar) {
        this.translationScalar = translationScalar;
        this.desiredRotationForRotationLockState = desiredRotation;
        setWantedState(WantedState.ROTATION_LOCK_SLOW_TRANSLATION);
    }

    /// Checks if robot is within tolerance of drive-to-point target
    public boolean isAtDriveToPointSetpoint() {
        var distance = desiredPoseForDriveToPoint
                .getTranslation()
                .minus(inputs.Pose.getTranslation())
                .getNorm();
        boolean isAtSetpoint = MathUtil.isNear(0.0, distance, translationErrorMarginForDriveToPoint);
        Logger.recordOutput("Drive/DriveToPoint/distanceFromEndpoint", distance);
        Logger.recordOutput("Drive/DriveToPoint/isAtSetpoint", isAtSetpoint);
        return isAtSetpoint;
    }

    /// Checks if robot is within default rotation tolerance (10degrees)
    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    /// Checks if robot is within specified rotation tolerance
    public boolean isAtDesiredRotation(double tolerance) {
        Logger.recordOutput("Swerve/RotationError", driveAtAngle.HeadingController.getPositionError());
        return Math.abs(driveAtAngle.HeadingController.getPositionError()) < tolerance;
    }

    /// Returns maximum translational velocity (m/s)
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /// Returns maximum angular velocity (rad/s)
    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public boolean isFollowingPath() {
        return systemState == SystemState.PATH_FOLLOWING || systemState == SystemState.DRIVING_TO_POINT;
    }

    /// Sets cached chassis speeds for rotation lock mode
    public void setTeleopChassisSpeed(ChassisSpeeds teleopChassisSpeed) {
        this.teleopChassisSpeeds = teleopChassisSpeed;
    }

    /// Returns MapleSim drivetrain instance if in simulation, null otherwise
    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof SwerveIOSim simIo) {
            return simIo.getMapleSimDrive();
        }
        return null;
    }

    public void setTrajectoryToFollow(PathPlannerTrajectory desiredTrajectory) {
        trajectoryToFollow = desiredTrajectory;
        this.wantedState = WantedState.FOLLOW_PATH;
    }

    private SwerveRequest seekingWheelchair() {
        /*
          This is a version of seeking to reduce strafing by only allowing the drive base to translate in its current intake direction
        */
        double rawX;
        double rawY;

        if (FieldConstants.isBlueAlliance()) {
            rawX = -MathUtil.applyDeadband(controllerX, DEADBAND);
            rawY = -MathUtil.applyDeadband(controllerY, DEADBAND);
        } else {
            rawX = MathUtil.applyDeadband(controllerX, DEADBAND);
            rawY = MathUtil.applyDeadband(controllerY, DEADBAND);
        }

        double magnitude = Math.hypot(rawX, rawY);

        if (magnitude < DEADBAND) {
            return idleRequest;
        }

        double xMagnitude = Math.copySign(rawX * rawX, rawX);
        double yMagnitude = Math.copySign(rawY * rawY, rawY);

        Rotation2d seekTargetRotation = new Rotation2d(rawX, rawY);

        // The robot's current heading determines the vx vy so there should be less strafing
        Rotation2d currentRotation = robotState.getFieldToRobotPose().getRotation();

        currentRotation = currentRotation.plus(Rotation2d.kPi);
        // currentRotation;

        // Try this instead of current velocity calculations:
        double speed = Math.hypot(xMagnitude, yMagnitude) * maxVelocity * teleopVelocityCoefficient;

        double vxField = speed * currentRotation.getCos();
        double vyField = speed * currentRotation.getSin();

        Logger.recordOutput("Drive/SeekingWheelchair/TargetHeading", seekTargetRotation);
        Logger.recordOutput(
                "Drive/SeekingWheelchair/HeadingError",
                seekTargetRotation.minus(currentRotation).getDegrees());

        return driveAtAngle
                .withVelocityX(vxField)
                .withVelocityY(vyField)
                .withCenterOfRotation(SEEKING_ROTATION_OFFSET)
                .withTargetDirection(seekTargetRotation);
    }

    private SwerveRequest pathFollowing() {
        var pose = robotState.getFieldToRobotPose();
        var sample = trajectoryToFollow.sample(trajectoryTimer.get());
        var targetSpeeds = new ChassisSpeeds(
                sample.fieldSpeeds.vxMetersPerSecond,
                sample.fieldSpeeds.vyMetersPerSecond,
                sample.fieldSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("Subsystems/Drive/PathPlanner/TimerValue", trajectoryTimer.get());
        Logger.recordOutput("Subsystems/Drive/PathPlanner/TotalTime", trajectoryToFollow.getTotalTime());

        targetSpeeds.vxMetersPerSecond += trajectoryXController.calculate(pose.getX(), sample.pose.getX());
        targetSpeeds.vyMetersPerSecond += trajectoryYController.calculate(pose.getY(), sample.pose.getY());
        targetSpeeds.omegaRadiansPerSecond += trajectoryThetaController.calculate(
                pose.getRotation().getRadians(), sample.pose.getRotation().getRadians());

        Logger.recordOutput(
                "Subsystems/Drive/PathPlanner/CalculatedVelocityX",
                trajectoryXController.calculate(pose.getX(), sample.pose.getX()));
        Logger.recordOutput(
                "Subsystems/Drive/PathPlanner/CalculatedVelocityY",
                trajectoryYController.calculate(pose.getY(), sample.pose.getY()));

        Logger.recordOutput("Subsystems/Drive/PathPlanner/sample/DesiredPose", sample.pose);
        Logger.recordOutput(
                "Subsystems/Drive/PathPlanner/sample/ErrorOfDesiredPoseAndActualPose", sample.pose.minus(pose));
        Logger.recordOutput("Subsystems/Drive/PathPlanner/sample/DesiredChassisSpeeds", targetSpeeds);
        Logger.recordOutput(
                "Subsystems/Drive/PathPlanner/sample/ModuleForcesX", sample.feedforwards.robotRelativeForcesXNewtons());
        Logger.recordOutput(
                "Subsystems/Drive/PathPlanner/sample/ModuleForcesY", sample.feedforwards.robotRelativeForcesYNewtons());

        return new SwerveRequest.ApplyFieldSpeeds()
                .withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.feedforwards.robotRelativeForcesX())
                .withWheelForceFeedforwardsY(sample.feedforwards.robotRelativeForcesY())
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    }

    public double getRobotDistanceFromPathPlannerEndpoint(PathPlannerTrajectory trajectoryToCheck) {
        Logger.recordOutput("PathPlanner/TrajectoryEndPose", trajectoryToCheck.getEndState().pose);
        var distance = Math.abs(trajectoryToCheck
                .getEndState()
                .pose
                .minus(robotState.getFieldToRobotPose())
                .getTranslation()
                .getNorm());
        Logger.recordOutput("PathPlanner/DistanceToEndOfTrajectory", distance);
        return distance;
    }

    /**
     * Reset the module
     * s so they're pointing at a heading of zero degrees.
     */
    public void setModulePositions(Rotation2d[] positions) {
        desiredModulePositions = positions;
        setWantedState(WantedState.SET_MODULE_POSITIONS);
    }

    public boolean isSettingModulePositions() {
        return wantedState == WantedState.SET_MODULE_POSITIONS || systemState == SystemState.SETTING_MODULE_POSITIONS;
    }

    private boolean areModulesAtPosition() {
        return moduleInputs.length
                == IntStream.range(0, moduleInputs.length)
                        .filter(index -> Math.abs(moduleInputs[index].steerPosition.getDegrees()
                                        - desiredModulePositions[index].getDegrees())
                                <= Constants.DriveConstants.resetModulesToleranceInDegrees)
                        .count();
    }

    // region Set [stator] current limit
    public void setStatorCurrentLimit(double currentLimit) {
        statorCurrentLimit = currentLimit;
        setWantedState(WantedState.SET_STATOR_CURRENT_LIMIT);
    }
    // endregion

    // region System characterization
    public SysIdRoutine getSysIdRoutine(final WantedState wantedState) {
        return switch (wantedState) {
            case SYS_ID_TRANSLATION -> translationSysIdRoutine;
            case SYS_ID_STEER -> steerSysIdRoutine;
            case SYS_ID_ROTATION -> rotationSysIdRoutine;
            default ->
                throw new RuntimeException("WantedState %s is not supported for SysId".formatted(wantedState.name()));
        };
    }

    private void sysIdStateConsumer(final SysIdRoutineLog.State state) {
        SignalLogger.writeString("state", state.toString());
    }

    private void voltageConsumer(final Voltage voltage) {
        final SwerveRequest controlRequest =
                switch (wantedState) {
                    case SYS_ID_TRANSLATION -> new SwerveRequest.SysIdSwerveTranslation().withVolts(voltage);
                    case SYS_ID_ROTATION ->
                        new SwerveRequest.SysIdSwerveRotation()
                                .withRotationalRate(voltage.timesConversionFactor(
                                        SystemCharacterizationConstants.VOLTAGE_TO_ROTATION_CONVERSION_FACTOR));
                    case SYS_ID_STEER -> new SwerveRequest.SysIdSwerveSteerGains().withVolts(voltage);
                    default -> new SwerveRequest.Idle();
                };

        io.setSwerveState(controlRequest);
    }
    // endregion
}
