package org.frc2910.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.util.StatusSignalRefresher;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.stream.Stream;

/**
 * CTRE Phoenix 6 hardware implementation for swerve drive.
 *
 * <p>Features: batch signal refresh (250Hz odometry), steer rate computation, zero-allocation updates.
 */
public class SwerveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements SwerveIO {
    private RobotState robotState;

    private interface ModuleSignalRefresher {
        void refreshSignals(ModuleIOInputs inputs);
    }

    private final Map<Integer, ModuleSignalRefresher> signalRefreshFunctions = new HashMap<>();

    /**
     * Yaw angular velocity from Pigeon 2 (250Hz, for gyro slip detection).
     */
    private final StatusSignal<AngularVelocity> angularYawVelocity;

    /**
     * X-axis linear acceleration from Pigeon 2.
     */
    private final StatusSignal<LinearAcceleration> accelerationX;

    /**
     * Y-axis linear acceleration from Pigeon 2.
     */
    private final StatusSignal<LinearAcceleration> accelerationY;

    /**
     * Z-axis linear acceleration from Pigeon 2 (250Hz, for airborne detection).
     */
    private final StatusSignal<LinearAcceleration> accelerationZ;

    /**
     * Labels used for logging
     **/
    private final String[] moduleNames = {"Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR"};

    /**
     * Constructs CTRE swerve hardware interface with signal batching and telemetry registration.
     *
     * @param constants       drivetrain constants (track width, wheelbase, etc.)
     * @param moduleConstants per-module constants (motor/encoder configs, coupling ratios)
     */
    @SuppressWarnings("unchecked")
    public SwerveIOCTRE(
            RobotState robotState,
            SwerveDrivetrainConstants constants,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>...
                    moduleConstants) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, constants, moduleConstants);
        final int moduleCount = getModules().length;

        this.robotState = robotState;

        // Register motor signals for batch refresh (8 signals per module)
        for (int i = 0; i < moduleCount; i++) {
            final var module = getModule(i);

            final TalonFX driveMotor = module.getDriveMotor();
            final TalonFX steerMotor = module.getSteerMotor();

            // Drive motor signals (supply current, stator current, voltage, temp)
            var driveSupplyCurrent = driveMotor.getSupplyCurrent();
            var driveStatorCurrent = driveMotor.getStatorCurrent();
            var driveAppliedVolts = driveMotor.getMotorVoltage();
            var driveTemperature = driveMotor.getDeviceTemp();

            // Steer motor signals (supply current, stator current, voltage, temp)
            var steerSupplyCurrent = steerMotor.getSupplyCurrent();
            var steerStatorCurrent = steerMotor.getStatorCurrent();
            var steerAppliedVolts = steerMotor.getMotorVoltage();
            var steerTemperature = steerMotor.getDeviceTemp();
            var steerVelocity = steerMotor.getVelocity();
            var steerPosition = steerMotor.getPosition();

            signalRefreshFunctions.put(i, (ModuleIOInputs inputs) -> {
                inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
                inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
                inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
                inputs.driveTemperature = driveTemperature.getValueAsDouble();

                inputs.steerSupplyCurrentAmps = steerSupplyCurrent.getValueAsDouble();
                inputs.steerStatorCurrentAmps = steerStatorCurrent.getValueAsDouble();
                inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
                inputs.steerTemperature = steerTemperature.getValueAsDouble();
                inputs.steerVelocity = steerVelocity.getValueAsDouble();
                inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
            });

            StatusSignalRefresher.getInstance()
                    .addStatusSignals(
                            driveSupplyCurrent,
                            driveStatorCurrent,
                            driveAppliedVolts,
                            driveTemperature,
                            steerSupplyCurrent,
                            steerStatorCurrent,
                            steerAppliedVolts,
                            steerTemperature,
                            steerVelocity,
                            steerPosition);
        }

        // Register Pigeon 2 gyro signals (yaw velocity + XYZ accelerations)
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();
        accelerationZ = getPigeon2().getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity, accelerationX, accelerationY, accelerationZ);
        StatusSignalRefresher.getInstance()
                .addStatusSignals(angularYawVelocity, accelerationX, accelerationY, accelerationZ);
    }

    // --------------------------------------------------------------------------
    //  SwerveIO IMPLEMENTATION
    // --------------------------------------------------------------------------

    /**
     * Updates sensor inputs from CTRE hardware (250Hz odometry thread + gyro signals).
     *
     * <p>Converts units to radians, computes module steer rates, and adds observations to RobotState.
     */
    @Override
    public void updateInputs(SwerveIOInputs inputs) {
        inputs.fromSwerveDriveState(this.getState());

        // Convert to radians and m/s^2 for internal use
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());
        inputs.yawAccelerationRadPerSecPerSec = accelerationZ.getValueAsDouble() * 9.80665; // g --> m/s^2

        // Convert robot-relative chassis speeds to field-relative
        var measuredFieldRelativeChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(inputs.Speeds, inputs.Pose.getRotation());

        // Add pose observation to RobotState for vision fusion
        robotState.addPoseObservation(
                Timer.getFPGATimestamp(),
                inputs.Pose,
                inputs.Speeds,
                measuredFieldRelativeChassisSpeeds,
                inputs.RawHeading.getRadians(),
                inputs.yawVelocityRadPerSec,
                inputs.yawAccelerationRadPerSecPerSec);
    }

    /**
     * Updates module motor telemetry using batch signal refresh for efficiency.
     *
     * @param inputs varargs array of module input objects to populate
     */
    @Override
    public void updateModuleInputs(ModuleIOInputs... inputs) {
        final int moduleCount = inputs.length;
        for (int i = 0; i < moduleCount; i++) {
            signalRefreshFunctions.get(i).refreshSignals(inputs[i]);
        }
    }

    /**
     * Returns swerve kinematics for coordinate transformations (chassis speeds <--> module states).
     */
    @Override
    public SwerveDriveKinematics getSwerveKinematics() {
        return getKinematics();
    }

    private String[][] outputNames;

    /**
     * Logs per-module telemetry: encoder angles, steering angles, and drive velocities.
     */
    @Override
    public void logModules(SwerveDriveState driveState) {
        if (driveState.ModuleStates == null) {
            return;
        }
        if (outputNames == null) {
            outputNames = new String[4][5];
            for (int i = 0; i < getModules().length; i++) {
                outputNames[i] = new String[5];
                outputNames[i][0] = moduleNames[i] + " Absolute Encoder Angle";
                outputNames[i][1] = moduleNames[i] + " Steering Angle";
                outputNames[i][2] = moduleNames[i] + " Target Steering Angle";
                outputNames[i][3] = moduleNames[i] + " Drive Velocity";
                outputNames[i][4] = moduleNames[i] + " Target Drive Velocity";
            }
        }
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(
                    outputNames[i][0],
                    getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(outputNames[i][1], driveState.ModuleStates[i].angle);
            Logger.recordOutput(outputNames[i][2], driveState.ModuleTargets[i].angle);
            Logger.recordOutput(outputNames[i][3], driveState.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(outputNames[i][4], driveState.ModuleTargets[i].speedMetersPerSecond);
        }
    }

    /**
     * Resets odometry to the specified pose.
     */
    @Override
    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    /**
     * Applies a swerve request to the drivetrain hardware.
     */
    @Override
    public void setSwerveState(SwerveRequest request) {
        super.setControl(request);
    }

    /**
     * Resets gyro heading to alliance-dependent forward (0degrees blue, 180degrees red).
     */
    @Override
    public void resetRotation() {
        this.resetRotation(FieldConstants.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);
    }

    /**
     * Resets gyro heading to a specific angle.
     */
    @Override
    public void resetToParamaterizedRotation(Rotation2d rotation2d) {
        this.resetRotation(rotation2d);
    }

    /**
     * Resets robot position on the field (preserves heading).
     */
    @Override
    public void resetRobotTranslation(Translation2d translation2d) {
        this.resetTranslation(translation2d);
    }

    @Override
    public void setStatorCurrentLimit(double currentLimit) {
        Stream.of(this.getModules()).map(SwerveModule::getDriveMotor).forEach(motor -> {
            final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            final TalonFXConfigurator configurator = motor.getConfigurator();

            configurator.refresh(motorConfiguration);

            configurator.apply(motorConfiguration.CurrentLimits.withStatorCurrentLimit(currentLimit));
        });
    }
}
