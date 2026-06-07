package org.frc2910.robot.config;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Configuration for a vision camera (Limelight) used on the robot.
 *
 * <p>Contains the name, mounting position and simple pipeline configuration options.
 * </p>
 */
public class CameraConfiguration {

    private final String name;
    private Pose3d mountingPosition = Pose3d.kZero;

    private double txScalarValue = 0;
    private final int enabledPipelineID = 0;
    private final int disabledPipelineID = 1;

    public CameraConfiguration(String name) {
        this.name = name;
    }

    public CameraConfiguration() {
        this("limelight");
    }

    public String getName() {
        return name;
    }

    public CameraConfiguration withTxScalarValue(double scaledTx) {
        this.txScalarValue = scaledTx;
        return this;
    }

    public double getTxScalarValue() {
        return txScalarValue;
    }

    public CameraConfiguration withMountingPosition(Pose3d pose) {
        this.mountingPosition = pose;

        return this;
    }

    public Pose3d getMountingPosition() {
        return mountingPosition;
    }

    public int getEnabledPipelineID() {
        return enabledPipelineID;
    }

    public int getDisabledPipelineID() {
        return disabledPipelineID;
    }
}
