package org.frc2910.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for swerve drive - supports CTRE, REV, simulation, or replay. */
public interface SwerveIO {
    /** Drive inputs from 250Hz odometry thread, plus gyro data for traction control. */
    @AutoLog
    class SwerveIOInputs extends SwerveDriveState {
        /** Yaw rate (rad/s) for slip detection and field-oriented drive */
        public double yawVelocityRadPerSec = 0.0;

        /** Yaw acceleration (rad/s^2) for airborne detection */
        public double yawAccelerationRadPerSecPerSec = 0.0;

        /** Module steer rates (rad/s) for steer-drive decoupling */
        public double[] steerRatesRadPerSec = new double[4];

        /** Copies CTRE odometry data into this inputs object */
        void fromSwerveDriveState(SwerveDriveState state) {
            this.Pose = state.Pose;
            this.RawHeading = state.RawHeading;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.Speeds = state.Speeds;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
            this.OdometryPeriod = state.OdometryPeriod;
            this.Timestamp = state.OdometryPeriod;
        }
    }

    /** Per-module motor telemetry (current, voltage, temperature). */
    @AutoLog
    class ModuleIOInputs {
        // Drive motor (Kraken X60)
        public boolean connected = false;
        public double driveSupplyCurrentAmps = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveTemperature = 0.0;

        // Steer motor
        public double steerSupplyCurrentAmps = 0.0;
        public double steerStatorCurrentAmps = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerTemperature = 0.0;
        public double steerVelocity = 0.0;
        public Rotation2d steerPosition = Rotation2d.kZero;
    }

    // ---- Input Reading ----

    /** Updates sensor inputs from hardware (called once per robot loop). */
    void updateInputs(SwerveIOInputs inputs);

    /** Updates module motor current, voltage, and temperature telemetry. */
    void updateModuleInputs(ModuleIOInputs... moduleInputs);

    /** Logs detailed module telemetry for debugging. */
    void logModules(SwerveDriveState driveState);

    /** Returns the swerve kinematics for coordinate transformations. */
    SwerveDriveKinematics getSwerveKinematics();

    /** Applies commanded module states to hardware (no-op for simulation/replay). */
    default void setSwerveState(SwerveRequest request) {}

    /** Resets odometry to the specified pose. */
    void resetOdometry(Pose2d pose);

    /** Resets gyro heading to zero (forward). */
    default void resetRotation() {}

    /** Resets gyro heading to a specific angle. */
    default void resetToParamaterizedRotation(Rotation2d rotation2d) {}

    /** Resets robot translation position on the field. */
    default void resetRobotTranslation(Translation2d translation2d) {}

    default void setStatorCurrentLimit(double currentLimit) {}
}
