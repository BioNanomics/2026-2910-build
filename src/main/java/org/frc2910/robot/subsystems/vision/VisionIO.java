package org.frc2910.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    default void updateInputs(VisionIOInputs inputs) {}

    default String getLimelightName() {
        return "";
    }

    @AutoLog
    class VisionIOInputs {
        public boolean isEnabled = false;
        public boolean hasTargets = false;
        public Rotation2d horizontalAngleToTarget = new Rotation2d();
        public int tagId = -1;
        public double targetHeight;
        public double distanceToTagMeters;
        public Rotation2d angleEncompassingTag;
        public Pose2d robotPoseBasedOffDistanceCalcAndTagLocation;
        public Pose2d robotPoseFromCamera;
        public Temperature temperature;
        public double pipelineId = -1.;
    }
}
