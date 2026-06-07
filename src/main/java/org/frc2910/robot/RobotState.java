package org.frc2910.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc2910.robot.constants.FieldConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

@SuppressWarnings("unused")
public class RobotState {
    private String gameSpecificMessage = null;

    public record AprilTagObservation(String cameraName, int tagId, Pose2d robotPoseFromCamera) {}

    // Kinematic Frames
    // Robot's pose in field coordinates over time
    private final List<AprilTagObservation> aprilTagObservations = new ArrayList<>();

    // Robot's pose in field coordinates over time
    private Pose2d fieldToRobot = Pose2d.kZero;
    private double lastTimestamp = Double.MIN_NORMAL;

    // Current field-relative chassis speeds (measured from encoders)
    private ChassisSpeeds measuredFieldRelativeChassisSpeeds = new ChassisSpeeds();

    public RobotState() {}

    public void addPoseObservation(
            double timestamp,
            Pose2d robotPose,
            ChassisSpeeds robotRelativeChassisSpeeds,
            ChassisSpeeds fieldRelativeChassisSpeeds,
            double yaw,
            double yawRate,
            double yawAccel) {
        updateRobotPoseIfNewer(timestamp, robotPose);
        this.measuredFieldRelativeChassisSpeeds = fieldRelativeChassisSpeeds;
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        updateRobotPoseIfNewer(timestamp, pose);
    }

    /// Returns the Current Robot Pose from Field Speeds
    public Pose2d getFieldToRobotPose() {
        return fieldToRobot;
    }

    /// Returns the Current Field Relative Chassis Speeds
    public ChassisSpeeds getMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds;
    }

    /**
     * Store the game-specific message (from FMS). Use {@link #getGameSpecificMessage()} to read.
     *
     * @param gameSpecificMessage the message to store (or null to clear)
     */
    public void setGameSpecificMessage(String gameSpecificMessage) {
        this.gameSpecificMessage = gameSpecificMessage;
    }

    public Optional<String> getGameSpecificMessage() {
        return Optional.ofNullable(gameSpecificMessage);
    }

    public void addVisionObservation(AprilTagObservation... observations) {
        aprilTagObservations.clear();
        aprilTagObservations.addAll(List.of(observations));
    }

    public List<AprilTagObservation> getVisionObservation() {
        return aprilTagObservations;
    }

    /**
     * Returns whether the stored game-specific message indicates the robot "won" auto
     * based on alliance-specific encoding (B/R).
     *
     * @return true if the message indicates a win for this alliance
     */
    public boolean wonAuto() {
        return Optional.ofNullable(gameSpecificMessage)
                .map(message -> (FieldConstants.isBlueAlliance() && message.equals("B"))
                        || (!FieldConstants.isBlueAlliance() && message.equals("R")))
                .orElse(false);
    }

    private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
        if (timestamp > lastTimestamp) {
            lastTimestamp = timestamp;
            fieldToRobot = pose;
        }
    }
}
