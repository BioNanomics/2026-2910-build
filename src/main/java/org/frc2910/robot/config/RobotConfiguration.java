package org.frc2910.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import java.util.List;

/**
 * Configuration interface providing access to all hardware configurations for a specific robot.
 *
 * <p>This interface abstracts away robot-specific configurations (e.g., REBLITZ vs simulation)
 * and provides a unified API for subsystems to access their motor configs, camera configs, etc.
 * </p>
 */
public interface RobotConfiguration {

    /**
     * Get the swerve drivetrain base frame constants (track width, wheelbase, etc.).
     *
     * @return the SwerveDrivetrainConstants
     */
    SwerveDrivetrainConstants getSwerveDrivetrainConstants();

    /**
     * Get the array of swerve module configurations (one per module: FL, FR, BL, BR).
     *
     * @return array of SwerveModuleConstants
     */
    SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] getModuleConstants();

    /**
     * Get the list of all vision camera configurations.
     *
     * @return list of CameraConfiguration instances
     */
    List<CameraConfiguration> getCameraConfigurations();

    /**
     * Get the intake subsystem configuration.
     *
     * @return the IntakeConfiguration
     */
    IntakeConfiguration getIntakeConfiguration();

    /**
     * Get the hopper/feeder subsystem configuration.
     *
     * @return the HopperConfiguration
     */
    HopperConfiguration getFeederConfiguration();

    /**
     * Get the shooter subsystem configuration.
     *
     * @return the ShooterConfiguration
     */
    ShooterConfiguration getShooterConfiguration();

    /**
     * Get the CANrange sensor configuration for the hopper.
     *
     * @return the hopper RangeConfiguration
     */
    RangeConfiguration getHopperRangeConfiguration();

    /**
     * Get the CANrange sensor configuration for the feeder.
     *
     * @return the feeder RangeConfiguration
     */
    RangeConfiguration getFeederRangeConfiguration();

    /**
     * Factory method to retrieve the appropriate configuration instance for a robot identity.
     *
     * @param robot the RobotIdentity indicating which robot configuration to load
     * @return the RobotConfiguration for the specified robot
     */
    static RobotConfiguration getRobotConfiguration(RobotIdentity robot) {
        return switch (robot) {
            case REBLITZ ->
                // ignore warning here this is set as such to make implementing robot switching logic easier to see
                new ReBlitz();
            default -> new ReBlitz();
        };
    }
}
