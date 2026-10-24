package org.frc2910.robot.config;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.frc2910.robot.subsystems.base.roller.RollerMotorSubsystemConfig;
import org.frc2910.robot.subsystems.base.roller.RollerWithFollowersSubsystemConfig;
import org.frc2910.robot.subsystems.base.servo.ServoMotorSubsystemConfig;
import org.frc2910.robot.util.CanDeviceId;
import org.frc2910.robot.util.DelayedBatchSignalLogger;

import java.util.List;

/**
 * Robot configuration implementation for the ReBlitz robot.
 *
 * <p>Contains all CAN device IDs, motor configurations, swerve module parameters, and subsystem
 * settings specific to the ReBlitz hardware.
 */
@SuppressWarnings({"UnusedVariable", "UnusedPrivateField"})
public class ReBlitz implements RobotConfiguration {
    /*
        All of these constants have been lifted directly from Thor and should be changed accordingly to match with
        CompBot
    */

    // Swerve values below are taken from the Tuner X generated TunerConstants (advkit)
    private static final String SWERVE_CANBUS_NAME = "CANivore";

    private static final CanDeviceId GYRO = new CanDeviceId(5, SWERVE_CANBUS_NAME);

    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
            .withKP(1.178)
            .withKI(0)
            .withKD(0)
            .withKS(0.042)
            .withKV(0.799)
            .withKA(0.063);
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
            .withKP(100)
            .withKI(0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(2.49)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // Swerve Modules
    private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(7, SWERVE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(8, SWERVE_CANBUS_NAME);
    private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(2, SWERVE_CANBUS_NAME);

    private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(3, SWERVE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(4, SWERVE_CANBUS_NAME);
    private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(3, SWERVE_CANBUS_NAME);

    private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(5, SWERVE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(6, SWERVE_CANBUS_NAME);
    private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(1, SWERVE_CANBUS_NAME);

    private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(1, SWERVE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(2, SWERVE_CANBUS_NAME);
    private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(4, SWERVE_CANBUS_NAME);

    /**
     * Wheel radius in meters. Accuracy in these measurements affects wheel odometry which measures
     * distance as a function of the number of rotations * wheel circumference.
     */
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

    /** The number of teeth on the drive motor pinion gear. */
    private static final double DRIVE_MOTOR_PINION_TEETH_COUNT = 12.0;

    /**
     * The coupled gear ratio between the CanCoder and the drive motor. Every 1 rotation of the steer
     * motor results in coupled ratio of drive turns. The first drive gear divided by the pinion gear
     */
    private static final double COUPLING_GEAR_RATIO = 54.0 / DRIVE_MOTOR_PINION_TEETH_COUNT;

    /**
     * Ratio between the drive motor shaft and the output shaft the wheel is mounted on. Mark 4i L3
     * ratio
     */
    private static final double DRIVE_GEAR_RATIO = COUPLING_GEAR_RATIO * (25.0 / 32.0) * (30.0 / 15.0);

    /** Ratio between the steer motor shaft and the steer output shaft. Mark 4i L3 ratio */
    private static final double STEER_GEAR_RATIO = 287.0 / 11.0;

    /**
     * Wheelbase length is the distance between the front and back wheels. Positive x values represent
     * moving towards the front of the robot
     */
    private static final double WHEELBASE_LENGTH_METERS = Units.inchesToMeters(22.25);

    /**
     * Wheel track width is the distance between the left and right wheels. Positive y values
     * represent moving towards the left of the robot.
     */
    private static final double WHEELBASE_WIDTH_METERS = Units.inchesToMeters(21.75);

    /** The maximum speed of the robot in meters per second. */
    @SuppressWarnings("unused")
    private static final double THEORETICAL_MAX_SPEED_METERS_PER_SECOND =
            6000.0 / 60.0 / DRIVE_GEAR_RATIO * WHEEL_RADIUS_METERS * 2 * Math.PI;

    // TODO: adjust max speed
    // This was observed via AdvantageScoped
    private static final double MAX_SPEED_METERS_PER_SECOND = 4.39;

    /**
     * The maximum angular speed of the robot in radians per second. If set to 0, the value is
     * calculated using the max speed in meters per second and the wheelbase radius.
     */

    // CANcoder offsets of the swerve modules - bevel gears pointing left of the robot
    private static final double FRONT_LEFT_STEER_OFFSET_ROTATIONS = -0.38330078125;

    private static final double FRONT_RIGHT_STEER_OFFSET_ROTATIONS = -0.1787109375;
    private static final double BACK_LEFT_STEER_OFFSET_ROTATIONS = -0.478271484375;
    private static final double BACK_RIGHT_STEER_OFFSET_ROTATIONS = -0.115234375;

    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private final SwerveDrivetrainConstants swerveDrivetrainConstants;
    private final SwerveModuleConstants[] moduleConstants;

    private final IntakeConfiguration intakeConfiguration;

    private final ShooterConfiguration shooterConfiguration;
    private final HopperConfiguration hopperConfiguration;
    private final List<CameraConfiguration> cameraConfigurations;
    private final RangeConfiguration hopperRangeConfiguration;
    private final RangeConfiguration feederRangeConfiguration;

    public ReBlitz() {
        // region Intake configuration
        intakeConfiguration = new IntakeConfiguration(
                new ServoMotorSubsystemConfig(
                        "IntakeDeploy", // TODO change all names to use slash directory when possible
                        new CanDeviceId(14, Constants.RIO_BUS_NAME),
                        "Degrees",
                        ServoMotorSubsystemConfig.createMotorConfiguration(
                                        Constants.IntakeConstants.INTAKE_GEAR_RATIO_WITH_UNIT_CONVERSIONS,
                                        Constants.IntakeConstants.INTAKE_MIN_ANGLE,
                                        Constants.IntakeConstants.INTAKE_MAX_ANGLE)
                                .withSlot0(new Slot0Configs()
                                        .withKP(Constants.IntakeConstants.INTAKE_SERVO_KP)
                                        .withKV(Constants.IntakeConstants.INTAKE_SERVO_KV))
                                .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(
                                                Constants.IntakeConstants.INTAKE_MOTION_MAGIC_VELOCITY)
                                        .withMotionMagicAcceleration(
                                                Constants.IntakeConstants.INTAKE_MOTION_MAGIC_ACCELERATION))
                                .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Coast)
                                        .withInverted(InvertedValue.Clockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(75.0)
                                        .withSupplyCurrentLimit(30.0)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimitEnable(true))),
                new RollerMotorSubsystemConfig(
                        "IntakeRightRoller",
                        new CanDeviceId(15, Constants.RIO_BUS_NAME),
                        "Rotations",
                        RollerMotorSubsystemConfig.createMotorConfiguration(1.0)
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(100.0)
                                        .withStatorCurrentLimit(120.0)
                                        .withSupplyCurrentLowerLimit(80.0)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Coast)
                                        .withInverted(InvertedValue.CounterClockwise_Positive))),
                new RollerMotorSubsystemConfig(
                        "IntakeLeftRoller",
                        new CanDeviceId(16, Constants.RIO_BUS_NAME),
                        "Rotations",
                        RollerMotorSubsystemConfig.createMotorConfiguration(1.0)
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(100.0)
                                        .withStatorCurrentLimit(120.0)
                                        .withSupplyCurrentLowerLimit(80.0)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Coast)
                                        .withInverted(InvertedValue.Clockwise_Positive))));
        // endregion

        // region shooter configuration
        var shooterLeftConfigs = RollerMotorSubsystemConfig.createMotorConfiguration(
                        Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO)
                .withVoltage(new VoltageConfigs().withPeakReverseVoltage(0.0))
                .withSlot0(new Slot0Configs()
                        .withKP(Constants.ShooterConstants.FLYWHEEL_KP)
                        .withKV(Constants.ShooterConstants.FLYWHEEL_KV))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimit(Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimit(Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT));

        var shooterRightConfigs = shooterLeftConfigs
                .clone()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Coast));

        RollerWithFollowersSubsystemConfig leaderShooterMotor = new RollerWithFollowersSubsystemConfig(
                "ShooterLeaderTopLeft", new CanDeviceId(9, Constants.RIO_BUS_NAME), "Rotations", shooterLeftConfigs);
        leaderShooterMotor.addFollowerConfig(leaderShooterMotor
        .new FollowerConfig(
                "ShooterBottomLeft",
                new CanDeviceId(10, Constants.RIO_BUS_NAME),
                "Rotations",
                shooterLeftConfigs,
                MotorAlignmentValue.Aligned));

        leaderShooterMotor.addFollowerConfig(leaderShooterMotor
        .new FollowerConfig(
                "ShooterTopRight",
                new CanDeviceId(11, Constants.RIO_BUS_NAME),
                "Rotations",
                shooterRightConfigs,
                MotorAlignmentValue.Opposed));

        leaderShooterMotor.addFollowerConfig(leaderShooterMotor
        .new FollowerConfig(
                "ShooterBottomRight",
                new CanDeviceId(12, Constants.RIO_BUS_NAME),
                "Rotations",
                shooterRightConfigs,
                MotorAlignmentValue.Opposed));

        shooterConfiguration = new ShooterConfiguration(
                leaderShooterMotor,
                new ServoMotorSubsystemConfig(
                        "HoodMotor",
                        new CanDeviceId(13, Constants.RIO_BUS_NAME),
                        "Degrees",
                        ServoMotorSubsystemConfig.createMotorConfiguration(
                                        Constants.ShooterConstants.HOOD_GEAR_RATIO_WITH_UNIT_CONVERSIONS,
                                        Constants.ShooterConstants.HOOD_MIN_ANGLE,
                                        Constants.ShooterConstants.HOOD_MAX_ANGLE)
                                .withSlot0(new Slot0Configs()
                                        .withKP(Constants.ShooterConstants.HOOD_KP)
                                        .withKV(Constants.ShooterConstants.HOOD_KV))
                                .withMotionMagic(new MotionMagicConfigs()
                                        .withMotionMagicCruiseVelocity(
                                                Constants.ShooterConstants.HOOD_MOTION_MAGIC_VELOCITY)
                                        .withMotionMagicAcceleration(
                                                Constants.ShooterConstants.HOOD_MOTION_MAGIC_ACCELERATION))
                                .withMotorOutput(new MotorOutputConfigs()
                                        .withInverted(InvertedValue.Clockwise_Positive)
                                        .withNeutralMode(NeutralModeValue.Brake))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(20.0)
                                        .withStatorCurrentLimit(60.0)
                                        .withStatorCurrentLimitEnable(true)
                                        .withSupplyCurrentLimitEnable(true))));
        // endregion

        // region Feeder configuration
        var rightHopperMotorConfiguration = RollerMotorSubsystemConfig.createMotorConfiguration(
                        Constants.HopperConstants.HOPPER_GEAR_RATIO)
                .withSlot0(new Slot0Configs().withKP(Constants.HopperConstants.HOPPER_KP))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80.0)
                        .withSupplyCurrentLimit(40.0)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimitEnable(true));

        var leftHopperMotorConfiguration = rightHopperMotorConfiguration
                .clone()
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake));

        hopperConfiguration = new HopperConfiguration(
                new RollerMotorSubsystemConfig(
                        "HopperFrontLeftMotor",
                        new CanDeviceId(17, Constants.RIO_BUS_NAME),
                        "Rotations",
                        leftHopperMotorConfiguration),
                new RollerMotorSubsystemConfig(
                        "HopperFrontRightMotor",
                        new CanDeviceId(21, Constants.RIO_BUS_NAME),
                        "Rotations",
                        rightHopperMotorConfiguration),
                new RollerMotorSubsystemConfig(
                        "HopperBackRightMotor",
                        new CanDeviceId(22, Constants.RIO_BUS_NAME),
                        "Rotations",
                        rightHopperMotorConfiguration),
                new RollerMotorSubsystemConfig(
                        "HopperBackLeftMotor",
                        new CanDeviceId(23, Constants.RIO_BUS_NAME),
                        "Rotations",
                        leftHopperMotorConfiguration));

        hopperRangeConfiguration = new RangeConfiguration(
                "HopperRange",
                new CanDeviceId(0, Constants.RIO_BUS_NAME),
                new CANrangeConfiguration()
                        .withProximityParams(new ProximityParamsConfigs()
                                .withProximityHysteresis(0.01)
                                .withProximityThreshold(0.57)
                                .withMinSignalStrengthForValidMeasurement(3500)));

        feederRangeConfiguration = new RangeConfiguration(
                "FeederRange",
                new CanDeviceId(1, Constants.RIO_BUS_NAME),
                new CANrangeConfiguration()
                        .withProximityParams(new ProximityParamsConfigs()
                                .withProximityHysteresis(0.01)
                                .withProximityThreshold(0.57)
                                .withMinSignalStrengthForValidMeasurement(3500)));
        // endregion

        // Tuner X leaves Pigeon 2 configs null, so whatever is already on the device is kept
        swerveDrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(GYRO.getBusName())
                .withPigeon2Id(GYRO.getDeviceNumber())
                .withPigeon2Configs(null);

        // Slip current determines stator limits
        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 40.0;

        var steerConfigs = new TalonFXConfiguration();
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfigs.CurrentLimits.SupplyCurrentLimit = 20.0;
        steerConfigs.CurrentLimits.StatorCurrentLimit = 70.0;

        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber() + "-Name", "FrontLeftDrive");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + FRONT_LEFT_STEER_MOTOR.getDeviceNumber() + "-Name", "FrontLeftSteer");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/CANcoder-" + FRONT_LEFT_STEER_ENCODER.getDeviceNumber() + "-Name",
                        "FrontLeftCANCoder");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + FRONT_RIGHT_DRIVE_MOTOR.getDeviceNumber() + "-Name", "FrontRightDrive");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + FRONT_RIGHT_STEER_MOTOR.getDeviceNumber() + "-Name", "FrontRightSteer");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/CANcoder-" + FRONT_RIGHT_STEER_ENCODER.getDeviceNumber() + "-Name",
                        "FrontRightCANCoder");
        DelayedBatchSignalLogger.getInstance()
                .writeString("Phoenix6/TalonFX-" + BACK_LEFT_DRIVE_MOTOR.getDeviceNumber() + "-Name", "BackLeftDrive");
        DelayedBatchSignalLogger.getInstance()
                .writeString("Phoenix6/TalonFX-" + BACK_LEFT_STEER_MOTOR.getDeviceNumber() + "-Name", "BackLeftSteer");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/CANcoder-" + BACK_LEFT_STEER_ENCODER.getDeviceNumber() + "-Name", "BackLeftCANCoder");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + BACK_RIGHT_DRIVE_MOTOR.getDeviceNumber() + "-Name", "BackRightDrive");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-" + BACK_RIGHT_STEER_MOTOR.getDeviceNumber() + "-Name", "BackRightSteer");
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/CANcoder-" + BACK_RIGHT_STEER_ENCODER.getDeviceNumber() + "-Name",
                        "BackRightCANCoder");

        moduleConstants = new SwerveModuleConstants[4];
        moduleConstants[0] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(INVERT_LEFT_SIDE)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEELBASE_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.2)
                .withSteerFrictionVoltage(0.2)
                .withDriveInertia(0.035)
                .withSteerInertia(0.01)
                .withSlipCurrent(100.0)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[1] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(FRONT_RIGHT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(FRONT_RIGHT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(FRONT_RIGHT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(INVERT_RIGHT_SIDE)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(FRONT_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEELBASE_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.2)
                .withSteerFrictionVoltage(0.2)
                .withDriveInertia(0.035)
                .withSteerInertia(0.01)
                .withSlipCurrent(100.0)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[2] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(BACK_LEFT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(BACK_LEFT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(BACK_LEFT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(INVERT_LEFT_SIDE)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_LEFT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(WHEELBASE_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.2)
                .withSteerFrictionVoltage(0.2)
                .withDriveInertia(0.035)
                .withSteerInertia(0.01)
                .withSlipCurrent(100.0)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        moduleConstants[3] = new SwerveModuleConstants<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(BACK_RIGHT_DRIVE_MOTOR.getDeviceNumber())
                .withSteerMotorId(BACK_RIGHT_STEER_MOTOR.getDeviceNumber())
                .withEncoderId(BACK_RIGHT_STEER_ENCODER.getDeviceNumber())
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLING_GEAR_RATIO)
                .withDriveMotorInverted(INVERT_RIGHT_SIDE)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(BACK_RIGHT_STEER_OFFSET_ROTATIONS)
                .withLocationX(-WHEELBASE_LENGTH_METERS / 2)
                .withLocationY(-WHEELBASE_WIDTH_METERS / 2)
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(DRIVE_GAINS)
                .withSteerMotorGains(STEER_GAINS)
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveConfigs)
                .withSteerMotorInitialConfigs(steerConfigs)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withDriveFrictionVoltage(0.2)
                .withSteerFrictionVoltage(0.2)
                .withDriveInertia(0.035)
                .withSteerInertia(0.01)
                .withSlipCurrent(100.0)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(MAX_SPEED_METERS_PER_SECOND)
                .withWheelRadius(WHEEL_RADIUS_METERS);

        cameraConfigurations = List.of(new CameraConfiguration()
                .withMountingPosition(new Pose3d(
                        new Translation3d(
                                Units.inchesToMeters(-13.25), Units.inchesToMeters(0), Units.inchesToMeters(15.5)),
                        new Rotation3d(0, 15, 0)))
                .withTxScalarValue(1.21));
    }

    @Override
    public SwerveDrivetrainConstants getSwerveDrivetrainConstants() {
        return swerveDrivetrainConstants;
    }

    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            getModuleConstants() {
        return moduleConstants;
    }

    @Override
    public List<CameraConfiguration> getCameraConfigurations() {
        return cameraConfigurations;
    }

    @Override
    public IntakeConfiguration getIntakeConfiguration() {
        return intakeConfiguration;
    }

    @Override
    public ShooterConfiguration getShooterConfiguration() {
        return shooterConfiguration;
    }

    @Override
    public HopperConfiguration getFeederConfiguration() {
        return hopperConfiguration;
    }

    @Override
    public RangeConfiguration getHopperRangeConfiguration() {
        return hopperRangeConfiguration;
    }

    @Override
    public RangeConfiguration getFeederRangeConfiguration() {
        return feederRangeConfiguration;
    }
}
