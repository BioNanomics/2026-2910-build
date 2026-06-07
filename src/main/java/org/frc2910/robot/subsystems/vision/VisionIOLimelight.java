package org.frc2910.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Temperature;
import org.frc2910.robot.RobotContainer;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.config.CameraConfiguration;
import org.frc2910.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Optional;

import static edu.wpi.first.units.Units.Celsius;

public class VisionIOLimelight implements VisionIO {

    /**
     * When limelight tx was compared to robot rotation from the gyro, it was observed that they did not scale at the same rate. Assuming the Pigeon2 scales correctly, this means the limelight's angle scaling is incorrect. Because the difference scaled linearly, we found out the ratio between the limelight angle and robot angle and applied it to the tx angle.
     */
    private final NetworkTableEntry validEntry;

    private final RobotContainer container;

    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry cornerEntries; // Layout is {0x,0y,1x,1y,etc.}

    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private final NetworkTableEntry throttleSetEntry;

    private final NetworkTableEntry getPipeEntry;

    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private final NetworkTableEntry pipelineEntry;

    private final NetworkTableEntry hwEntry;

    private final ArrayList<Pair<Double, Double>> cornerListPairs;

    private final String limelightName;
    private final Translation2d cameraToRobotCenter;
    private final double txScalarValue;

    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private final int enabledPipelineID;

    private final int disabledPipelineID;
    private double tagHeight;
    private final RobotState robotState;

    public VisionIOLimelight(CameraConfiguration configuration, RobotContainer container, RobotState robotState) {
        this.container = container;
        this.robotState = robotState;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        limelightName = configuration.getName();
        cameraToRobotCenter =
                configuration.getMountingPosition().getTranslation().toTranslation2d();
        txScalarValue = configuration.getTxScalarValue();
        enabledPipelineID = configuration.getEnabledPipelineID();
        disabledPipelineID = configuration.getDisabledPipelineID();

        // Getting data from Network Tables
        validEntry = inst.getTable(limelightName).getEntry("tv");
        txEntry = inst.getTable(limelightName).getEntry("tx");
        tagIdEntry = inst.getTable(limelightName).getEntry("tid");
        cornerEntries = inst.getTable(limelightName).getEntry("tcornxy");
        throttleSetEntry = inst.getTable(limelightName).getEntry("throttle_set");
        getPipeEntry = inst.getTable(limelightName).getEntry("getpipe");
        pipelineEntry = inst.getTable(limelightName).getEntry("pipeline");
        hwEntry = inst.getTable(limelightName).getEntry("hw");

        cornerListPairs = new ArrayList<>();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.isEnabled = getPipeEntry.getInteger(0) != disabledPipelineID;
        inputs.pipelineId = getPipeEntry.getDouble(-1.);
        inputs.horizontalAngleToTarget = Rotation2d.fromDegrees(txEntry.getDouble(0.0));
        inputs.hasTargets = validEntry.getDouble(0.0) == 1.0; // this was ty != 0 last year - double check
        inputs.tagId = (int) tagIdEntry.getDouble(-1);
        this.getCameraTemperature().ifPresent(temperature -> inputs.temperature = temperature);
        cornerListPairs.clear();
        var x = cornerEntries.getDoubleArray(new double[0]);
        if (x.length >= 8) {
            for (int i = 0; i < 7; i += 2) {
                cornerListPairs.add(new Pair<>(x[i], x[i + 1]));
            }
            sortCorners2(cornerListPairs);
        }

        Logger.recordOutput(limelightName + "/Number of corners", cornerListPairs.size());

        if (inputs.hasTargets && cornerListPairs.size() >= 4) {
            var sortedCornerListPairs = sortCorners2(cornerListPairs);
            inputs.targetHeight = calculateTargetHeightInPixels(sortedCornerListPairs);
            inputs.distanceToTagMeters = container.getVisionInterpolation().getInterpolatedScaledDistance(tagHeight);
            Logger.recordOutput(
                    limelightName + "/sortedCornerListX",
                    sortedCornerListPairs.stream().mapToDouble(Pair::getFirst).toArray());
        } else {
            inputs.targetHeight = -1;
            inputs.distanceToTagMeters = -1;
        }

        if (inputs.hasTargets && inputs.distanceToTagMeters != -1 && inputs.tagId > 0 && inputs.tagId <= 32) {
            inputs.robotPoseBasedOffDistanceCalcAndTagLocation =
                    calculateRobotPose(inputs.tagId, inputs.distanceToTagMeters, inputs.horizontalAngleToTarget);
        } else {
            inputs.robotPoseBasedOffDistanceCalcAndTagLocation = null;
        }
    }

    @Override
    public String getLimelightName() {
        return limelightName;
    }

    private double calculateTargetHeightInPixels(Collection<Pair<Double, Double>> sortedCorners) {

        if (sortedCorners.size() >= 8) {
            return -1;
        }
        var pixelHeight = sortedCorners.stream().toList();
        var leftHeight =
                Math.abs(pixelHeight.get(0).getSecond() - pixelHeight.get(1).getSecond());
        var rightHeight =
                Math.abs(pixelHeight.get(2).getSecond() - pixelHeight.get(3).getSecond());
        tagHeight = (leftHeight + rightHeight) / 2;
        Logger.recordOutput(limelightName + "/tagHeight", tagHeight);
        FieldConstants.FIELD_LAYOUT.getTagPose(18).ifPresent(pose -> {
            Logger.recordOutput("Tag", pose);
        });

        return tagHeight;
    }

    private Optional<Temperature> getCameraTemperature() {
        var hwData = hwEntry.getDoubleArray(new Double[0]);
        if (hwData.length == 0) {
            return Optional.empty();
        }

        return Optional.of(Celsius.of(hwData[0]));
    }

    private static Collection<Pair<Double, Double>> sortCorners2(Collection<Pair<Double, Double>> entries) {
        return entries.stream()
                .sorted(Comparator.comparingDouble(Pair::getFirst))
                .toList();
    }

    private Pose2d calculateRobotPose(final int tagId, double distanceToTagMeters, Rotation2d horizontalAngleToTarget) {
        var tagPose = FieldConstants.getTagPose(tagId).toPose2d();
        Logger.recordOutput(limelightName + "/tagPose", tagPose);

        var robotRotation = robotState.getFieldToRobotPose().getRotation();
        Logger.recordOutput(limelightName + "/robotRotation", robotRotation);

        var robotRotationWithLimelightCorrection = robotRotation.plus(Rotation2d.kZero); // limelightYaw
        Logger.recordOutput(
                limelightName + "/robotRotationWithLimelightCorrection", robotRotationWithLimelightCorrection);

        // If the ll is upside down this value is positive, else its negative.
        var scaledTx = Rotation2d.fromDegrees(
                horizontalAngleToTarget.div(txScalarValue).getDegrees());
        Logger.recordOutput(limelightName + "/scaledTX", scaledTx);

        var cameraToRobotCenter = this.cameraToRobotCenter.rotateBy(
                robotState.getFieldToRobotPose().getRotation());
        Logger.recordOutput(limelightName + "/cameraToRobotCenter", cameraToRobotCenter);

        var angleToTag = scaledTx.plus(robotRotationWithLimelightCorrection);
        Logger.recordOutput(limelightName + "/angleToTag", angleToTag);

        var translation = new Translation2d(distanceToTagMeters, angleToTag);
        Logger.recordOutput(limelightName + "/translation", translation);
        var translatedPose = tagPose.getTranslation().minus(translation);
        Logger.recordOutput(limelightName + "/translatedPose", translatedPose);
        var fieldRelativeRobotTranslation = translatedPose.plus(cameraToRobotCenter);
        Logger.recordOutput(limelightName + "/fieldRelativeRobotTranslation", fieldRelativeRobotTranslation);
        var fieldRelativeRobotPoseBasedOnVision = new Pose2d(fieldRelativeRobotTranslation, robotRotation);
        var cameraPose = new Pose2d(translatedPose, robotRotation);
        Logger.recordOutput(
                limelightName + "/fieldRelativeRobotPoseBasedOnVision", fieldRelativeRobotPoseBasedOnVision);
        Logger.recordOutput(limelightName + "/cameraPose", cameraPose);
        return fieldRelativeRobotPoseBasedOnVision;
    }
}
