package org.frc2910.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.*;

/**
 * Contains field geometry, coordinate positions, and alliance-relative pose lookups.
 *
 * <p>Provides the 2026 game field layout with AprilTag positions and getter methods to retrieve
 * alliance-specific robot waypoints, scoring positions, and sweep routes.
 * </p>
 */
public class FieldConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT;

    static {
        FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public static final double FIELD_WIDTH = FIELD_LAYOUT.getFieldWidth();
    public static final double FIELD_LENGTH = FIELD_LAYOUT.getFieldLength();

    public static final Translation3d RED_HUB = new Translation3d(11.915, 4.035, Units.inchesToMeters(72));
    public static final Translation3d BLUE_HUB = new Translation3d(4.626, 4.035, Units.inchesToMeters(72));

    // TODO blue_left_point is on the right of the blue side in sim? Test and adjust coordinate so name is correct
    private static final Translation2d BLUE_LEFT_POINT_TO_PASS_TO = new Translation2d(4.0, 2.5);
    private static final Translation2d BLUE_RIGHT_POINT_TO_PASS_TO =
            new Translation2d(BLUE_LEFT_POINT_TO_PASS_TO.getX(), FIELD_WIDTH - BLUE_LEFT_POINT_TO_PASS_TO.getY());
    private static final Translation2d RED_LEFT_POINT_TO_PASS_TO = new Translation2d(
            FIELD_LENGTH - BLUE_RIGHT_POINT_TO_PASS_TO.getX(), FIELD_WIDTH - BLUE_LEFT_POINT_TO_PASS_TO.getY());
    private static final Translation2d RED_RIGHT_POINT_TO_PASS_TO =
            new Translation2d(FIELD_LENGTH - BLUE_RIGHT_POINT_TO_PASS_TO.getX(), BLUE_LEFT_POINT_TO_PASS_TO.getY());

    // Auto locations
    private static final Pose2d LEFT_FEED_PRELOAD_STARTING_POSE_BLUE =
            new Pose2d(4.5, 7.6, Rotation2d.fromDegrees(90.0));
    private static final Pose2d LEFT_FEED_PRELOAD_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - LEFT_FEED_PRELOAD_STARTING_POSE_BLUE.getX(),
            FIELD_WIDTH - LEFT_FEED_PRELOAD_STARTING_POSE_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_FEED_PRELOAD_STARTING_POSE_BLUE.getRotation()));

    private static final Pose2d LEFT_FIRST_SWEEP_ENTRY_BLUE = new Pose2d(9.9, 7.0, Rotation2d.fromDegrees(90.0));
    private static final Pose2d LEFT_FIRST_SWEEP_ENTRY_RED = new Pose2d(
            FIELD_LENGTH - LEFT_FIRST_SWEEP_ENTRY_BLUE.getX(),
            FIELD_WIDTH - LEFT_FIRST_SWEEP_ENTRY_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_FIRST_SWEEP_ENTRY_BLUE.getRotation()));

    private static final Pose2d LEFT_FIRST_SWEEP_CENTERPOINT_BLUE = new Pose2d(8.1, 2.8, Rotation2d.fromDegrees(90.0));
    private static final Pose2d LEFT_FIRST_SWEEP_CENTERPOINT_RED = new Pose2d(
            FIELD_LENGTH - LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getX(),
            FIELD_WIDTH - LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getRotation()));

    private static final Pose2d LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND = new Pose2d(
            LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getX(),
            LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getY(),
            Rotation2d.fromDegrees(180.0));
    private static final Pose2d LEFT_FIRST_SWEEP_CENTERPOINT_RED_SECOND = new Pose2d(
            FIELD_LENGTH - LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getX(),
            FIELD_WIDTH - LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getY(),
            Rotation2d.k180deg.plus(LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getRotation()));

    private static final Pose2d LEFT_SECOND_SWEEP_ENTRY_BLUE = new Pose2d(7.2, 7.7, Rotation2d.fromDegrees(100.0));
    private static final Pose2d LEFT_SECOND_SWEEP_ENTRY_RED = new Pose2d(
            FIELD_LENGTH - LEFT_SECOND_SWEEP_ENTRY_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_SWEEP_ENTRY_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_SECOND_SWEEP_ENTRY_BLUE.getRotation()));

    private static final Pose2d LEFT_SECOND_SWEEP_CENTERPOINT_BLUE =
            new Pose2d(6.0, 3.3, Rotation2d.fromDegrees(100.0));
    private static final Pose2d LEFT_SECOND_SWEEP_CENTERPOINT_RED = new Pose2d(
            FIELD_LENGTH - LEFT_SECOND_SWEEP_CENTERPOINT_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_SWEEP_CENTERPOINT_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_SECOND_SWEEP_CENTERPOINT_BLUE.getRotation()));

    private static final Pose2d LEFT_SWEEP_EXIT_BLUE =
            new Pose2d(6.3, 5.3, Rotation2d.fromDegrees(-135.0)); // only second pass in test auto
    private static final Pose2d LEFT_SWEEP_EXIT_RED = new Pose2d(
            FIELD_LENGTH - LEFT_SWEEP_EXIT_BLUE.getX(),
            FIELD_WIDTH - LEFT_SWEEP_EXIT_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_SWEEP_EXIT_BLUE.getRotation()));

    private static final Pose2d LEFT_TRENCH_ENTER_BLUE = new Pose2d(2.8, 7.7, Rotation2d.fromDegrees(180.0));
    private static final Pose2d LEFT_TRENCH_ENTER_RED = new Pose2d(
            FIELD_LENGTH - LEFT_TRENCH_ENTER_BLUE.getX(),
            FIELD_WIDTH - LEFT_TRENCH_ENTER_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_TRENCH_ENTER_BLUE.getRotation()));

    // first and second for scoring rotation
    private static final Pose2d LEFT_FIRST_BUMP_SCORE_BLUE = new Pose2d(2.6, 5.6, Rotation2d.fromDegrees(-90.0));
    private static final Pose2d LEFT_FIRST_BUMP_SCORE_RED = new Pose2d(
            FIELD_LENGTH - LEFT_FIRST_BUMP_SCORE_BLUE.getX(),
            FIELD_WIDTH - LEFT_FIRST_BUMP_SCORE_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_FIRST_BUMP_SCORE_BLUE.getRotation()));

    private static final Pose2d LEFT_SECOND_BUMP_SCORE_BLUE = new Pose2d(
            LEFT_FIRST_BUMP_SCORE_BLUE.getX(), LEFT_FIRST_BUMP_SCORE_BLUE.getY(), Rotation2d.fromDegrees(-45.0));
    private static final Pose2d LEFT_SECOND_BUMP_SCORE_RED = new Pose2d(
            FIELD_LENGTH - LEFT_SECOND_BUMP_SCORE_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_BUMP_SCORE_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_SECOND_BUMP_SCORE_BLUE.getRotation()));

    private static final Pose2d LEFT_SECOND_SWEEP_CURVEPOINT_BLUE =
            new Pose2d(8.3, 4.8, Rotation2d.fromDegrees(-170.0));
    private static final Pose2d LEFT_SECOND_SWEEP_CURVEPOINT_RED = new Pose2d(
            FIELD_LENGTH - LEFT_SECOND_SWEEP_CURVEPOINT_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_SWEEP_CURVEPOINT_BLUE.getY(),
            Rotation2d.k180deg.plus(LEFT_SECOND_SWEEP_CURVEPOINT_BLUE.getRotation()));

    // RIGHT
    private static final Pose2d RIGHT_SECOND_SWEEP_CURVEPOINT_BLUE = new Pose2d(
            LEFT_SECOND_SWEEP_CURVEPOINT_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_SWEEP_CURVEPOINT_BLUE.getY(),
            LEFT_SECOND_SWEEP_CURVEPOINT_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_SECOND_SWEEP_CURVEPOINT_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_SECOND_SWEEP_CURVEPOINT_BLUE.getX(),
            FIELD_WIDTH - RIGHT_SECOND_SWEEP_CURVEPOINT_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_SECOND_SWEEP_CURVEPOINT_BLUE.getRotation()));

    private static final Pose2d RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE = new Pose2d(
            LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getX(),
            FIELD_WIDTH - LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getY(),
            LEFT_FIRST_SWEEP_CENTERPOINT_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_FIRST_SWEEP_CENTERPOINT_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE.getX(),
            FIELD_WIDTH - RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE.getRotation()));

    private static final Pose2d RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND = new Pose2d(
            LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getX(),
            FIELD_WIDTH - LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getY(),
            LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getRotation().times(-1.0));
    private static final Pose2d RIGHT_FIRST_SWEEP_CENTERPOINT_RED_SECOND = new Pose2d(
            FIELD_LENGTH - RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getX(),
            FIELD_WIDTH - RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getY(),
            Rotation2d.k180deg.plus(RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND.getRotation()));

    private static final Pose2d RIGHT_FIRST_BUMP_SCORE_BLUE = new Pose2d(
            LEFT_FIRST_BUMP_SCORE_BLUE.getX(),
            FIELD_WIDTH - LEFT_FIRST_BUMP_SCORE_BLUE.getY(),
            LEFT_FIRST_BUMP_SCORE_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_FIRST_BUMP_SCORE_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_FIRST_BUMP_SCORE_BLUE.getX(),
            FIELD_WIDTH - RIGHT_FIRST_BUMP_SCORE_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_FIRST_BUMP_SCORE_BLUE.getRotation()));

    private static final Pose2d RIGHT_SECOND_BUMP_SCORE_BLUE = new Pose2d(
            LEFT_SECOND_BUMP_SCORE_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_BUMP_SCORE_BLUE.getY(),
            LEFT_SECOND_BUMP_SCORE_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_SECOND_BUMP_SCORE_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_SECOND_BUMP_SCORE_BLUE.getX(),
            FIELD_WIDTH - RIGHT_SECOND_BUMP_SCORE_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_SECOND_BUMP_SCORE_BLUE.getRotation()));

    private static final Pose2d RIGHT_SWEEP_EXIT_BLUE = new Pose2d(
            LEFT_SWEEP_EXIT_BLUE.getX(),
            FIELD_WIDTH - LEFT_SWEEP_EXIT_BLUE.getY(),
            LEFT_SWEEP_EXIT_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_SWEEP_EXIT_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_SWEEP_EXIT_BLUE.getX(),
            FIELD_WIDTH - RIGHT_SWEEP_EXIT_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_SWEEP_EXIT_BLUE.getRotation()));

    private static final Pose2d RIGHT_TRENCH_ENTER_BLUE = new Pose2d(
            LEFT_TRENCH_ENTER_BLUE.getX(),
            FIELD_WIDTH - LEFT_TRENCH_ENTER_BLUE.getY(),
            LEFT_TRENCH_ENTER_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_TRENCH_ENTER_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_TRENCH_ENTER_BLUE.getX(),
            FIELD_WIDTH - RIGHT_TRENCH_ENTER_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_TRENCH_ENTER_BLUE.getRotation()));

    private static final Pose2d RIGHT_SECOND_SWEEP_ENTRY_BLUE = new Pose2d(
            LEFT_SECOND_SWEEP_ENTRY_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_SWEEP_ENTRY_BLUE.getY(),
            LEFT_SECOND_SWEEP_ENTRY_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_SECOND_SWEEP_ENTRY_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_SECOND_SWEEP_ENTRY_BLUE.getX(),
            FIELD_WIDTH - RIGHT_SECOND_SWEEP_ENTRY_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_SECOND_SWEEP_ENTRY_BLUE.getRotation()));

    private static final Pose2d RIGHT_SECOND_SWEEP_CENTERPOINT_BLUE = new Pose2d(
            LEFT_SECOND_SWEEP_CENTERPOINT_BLUE.getX(),
            FIELD_WIDTH - LEFT_SECOND_SWEEP_CENTERPOINT_BLUE.getY(),
            LEFT_SECOND_SWEEP_CENTERPOINT_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_SECOND_SWEEP_CENTERPOINT_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_SECOND_SWEEP_CENTERPOINT_BLUE.getX(),
            FIELD_WIDTH - RIGHT_SECOND_SWEEP_CENTERPOINT_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_SECOND_SWEEP_CENTERPOINT_BLUE.getRotation()));

    private static final Pose2d RIGHT_FEED_PRELOAD_STARTING_POSE_BLUE = new Pose2d(
            LEFT_FEED_PRELOAD_STARTING_POSE_BLUE.getX(),
            FIELD_WIDTH - LEFT_FEED_PRELOAD_STARTING_POSE_BLUE.getY(),
            LEFT_FEED_PRELOAD_STARTING_POSE_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_FEED_PRELOAD_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_FEED_PRELOAD_STARTING_POSE_BLUE.getX(),
            FIELD_WIDTH - RIGHT_FEED_PRELOAD_STARTING_POSE_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_FEED_PRELOAD_STARTING_POSE_BLUE.getRotation()));

    private static final Pose2d RIGHT_FIRST_SWEEP_ENTRY_BLUE = new Pose2d(
            LEFT_FIRST_SWEEP_ENTRY_BLUE.getX(),
            FIELD_WIDTH - LEFT_FIRST_SWEEP_ENTRY_BLUE.getY(),
            LEFT_FIRST_SWEEP_ENTRY_BLUE.getRotation().times(-1.0));
    private static final Pose2d RIGHT_FIRST_SWEEP_ENTRY_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_FIRST_SWEEP_ENTRY_BLUE.getX(),
            FIELD_WIDTH - RIGHT_FIRST_SWEEP_ENTRY_BLUE.getY(),
            Rotation2d.k180deg.plus(RIGHT_FIRST_SWEEP_ENTRY_BLUE.getRotation()));

    // CENTER
    private static final Pose2d CENTER_STARTING_POSE_BLUE = new Pose2d(3.6, 4.0, Rotation2d.kZero);
    private static final Pose2d CENTER_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - CENTER_STARTING_POSE_BLUE.getX(),
            FIELD_WIDTH - CENTER_STARTING_POSE_BLUE.getY(),
            Rotation2d.k180deg.plus(CENTER_STARTING_POSE_BLUE.getRotation()));

    private static final Pose2d DEPOT_RIGHT_BLUE = new Pose2d(0.65, 5.8, Rotation2d.fromDegrees(-20.0));
    private static final Pose2d DEPOT_RIGHT_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_RIGHT_BLUE.getX(),
            FIELD_WIDTH - DEPOT_RIGHT_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_RIGHT_BLUE.getRotation()));

    private static final Pose2d DEPOT_MIDDLE_BLUE = new Pose2d(1.5, 6.0, Rotation2d.kZero);
    private static final Pose2d DEPOT_MIDDLE_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_MIDDLE_BLUE.getX(),
            FIELD_WIDTH - DEPOT_MIDDLE_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_MIDDLE_BLUE.getRotation()));

    private static final Pose2d DEPOT_LEFT_BLUE = new Pose2d(0.65, 6.2, Rotation2d.fromDegrees(-10.0));
    private static final Pose2d DEPOT_LEFT_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_LEFT_BLUE.getX(),
            FIELD_WIDTH - DEPOT_LEFT_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_LEFT_BLUE.getRotation()));

    private static final Pose2d DEPOT_SCORING_POSE_BLUE = new Pose2d(1.4, 5.0, Rotation2d.fromDegrees(-20.0));
    private static final Pose2d DEPOT_SCORING_POSE_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_SCORING_POSE_BLUE.getX(),
            FIELD_WIDTH - DEPOT_SCORING_POSE_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_SCORING_POSE_BLUE.getRotation()));

    private static final Pose2d DEPOT_TRENCH_ENTER_INTERMEDIARY_BLUE =
            new Pose2d(1.5, 6.0, Rotation2d.fromDegrees(5.0));
    private static final Pose2d DEPOT_TRENCH_ENTER_INTERMEDIARY_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_TRENCH_ENTER_INTERMEDIARY_BLUE.getX(),
            FIELD_WIDTH - DEPOT_TRENCH_ENTER_INTERMEDIARY_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_TRENCH_ENTER_INTERMEDIARY_BLUE.getRotation()));

    private static final Pose2d DEPOT_TRENCH_ENTER_BLUE = new Pose2d(1.4, 7.7, Rotation2d.k180deg);
    private static final Pose2d DEPOT_TRENCH_ENTER_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_TRENCH_ENTER_BLUE.getX(),
            FIELD_WIDTH - DEPOT_TRENCH_ENTER_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_TRENCH_ENTER_BLUE.getRotation()));

    private static final Pose2d DEPOT_NZ_CENTER_BLUE = new Pose2d(8.3, 7.7, Rotation2d.k180deg);
    private static final Pose2d DEPOT_NZ_CENTER_RED = new Pose2d(
            FIELD_LENGTH - DEPOT_NZ_CENTER_BLUE.getX(),
            FIELD_WIDTH - DEPOT_NZ_CENTER_BLUE.getY(),
            Rotation2d.k180deg.plus(DEPOT_NZ_CENTER_BLUE.getRotation()));

    static List<Translation2d> bluePassingTranslations =
            Arrays.asList(BLUE_LEFT_POINT_TO_PASS_TO, BLUE_RIGHT_POINT_TO_PASS_TO);
    static List<Translation2d> redPassingTranslations =
            Arrays.asList(RED_LEFT_POINT_TO_PASS_TO, RED_RIGHT_POINT_TO_PASS_TO);

    /** Enumeration of known fixed translation locations on the field (trenches, uprights). */
    public enum KnownTranslations {
        LEFT_TRENCH,
        RIGHT_TRENCH,
        LEFT_UPRIGHT, // Uprights are not centered to hub on the field
        RIGHT_UPRIGHT,
    }

    /** Map of known translation points for the blue alliance. */
    public static final Map<KnownTranslations, Translation2d> blueKnownTranslationsMap =
            new EnumMap<>(KnownTranslations.class);

    static {
        blueKnownTranslationsMap.put(KnownTranslations.LEFT_TRENCH, new Translation2d(3.6, 6.3));
        blueKnownTranslationsMap.put(
                KnownTranslations.RIGHT_TRENCH,
                new Translation2d(
                        blueKnownTranslationsMap
                                .get(KnownTranslations.LEFT_TRENCH)
                                .getX(),
                        1.8));
        blueKnownTranslationsMap.put(KnownTranslations.LEFT_UPRIGHT, new Translation2d(0.95, 4.6));
        blueKnownTranslationsMap.put(
                KnownTranslations.RIGHT_UPRIGHT,
                new Translation2d(
                        blueKnownTranslationsMap
                                .get(KnownTranslations.LEFT_UPRIGHT)
                                .getX(),
                        2.76));
    }

    /** Map of known translation points for the red alliance. */
    public static final Map<KnownTranslations, Translation2d> redKnownTranslationsMap =
            new EnumMap<>(KnownTranslations.class);

    static {
        redKnownTranslationsMap.put(
                KnownTranslations.LEFT_TRENCH,
                new Translation2d(
                        FIELD_LENGTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.LEFT_TRENCH)
                                        .getX(),
                        FIELD_WIDTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.LEFT_TRENCH)
                                        .getY()));
        redKnownTranslationsMap.put(
                KnownTranslations.RIGHT_TRENCH,
                new Translation2d(
                        FIELD_LENGTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.RIGHT_TRENCH)
                                        .getX(),
                        FIELD_WIDTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.RIGHT_TRENCH)
                                        .getY()));
        redKnownTranslationsMap.put(
                KnownTranslations.LEFT_UPRIGHT,
                new Translation2d(
                        FIELD_LENGTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.LEFT_UPRIGHT)
                                        .getX(),
                        FIELD_WIDTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.LEFT_UPRIGHT)
                                        .getY()));
        redKnownTranslationsMap.put(
                KnownTranslations.RIGHT_UPRIGHT,
                new Translation2d(
                        FIELD_LENGTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.RIGHT_UPRIGHT)
                                        .getX(),
                        FIELD_WIDTH
                                - blueKnownTranslationsMap
                                        .get(KnownTranslations.RIGHT_UPRIGHT)
                                        .getY()));
    }

    public static Pose2d getLeftFirstSweepEntry(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_FIRST_SWEEP_ENTRY_BLUE : LEFT_FIRST_SWEEP_ENTRY_RED;
    }

    public static Pose2d getRightFirstSweepEntry(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_FIRST_SWEEP_ENTRY_BLUE : RIGHT_FIRST_SWEEP_ENTRY_RED;
    }

    public static Pose2d getLeftSecondSweepEntry(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_SECOND_SWEEP_ENTRY_BLUE : LEFT_SECOND_SWEEP_ENTRY_RED;
    }

    public static Pose2d getRightSecondSweepEntry(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_SECOND_SWEEP_ENTRY_BLUE : RIGHT_SECOND_SWEEP_ENTRY_RED;
    }

    public static Pose2d getLeftFirstSweepCenterpoint(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? LEFT_FIRST_SWEEP_CENTERPOINT_BLUE
                : LEFT_FIRST_SWEEP_CENTERPOINT_RED;
    }

    public static Pose2d getLeftFirstSweepCenterpointSecond(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? LEFT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND
                : LEFT_FIRST_SWEEP_CENTERPOINT_RED_SECOND;
    }

    public static Pose2d getRightFirstSweepCenterpoint(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE
                : RIGHT_FIRST_SWEEP_CENTERPOINT_RED;
    }

    public static Pose2d getRightFirstSweepCenterpointSecond(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? RIGHT_FIRST_SWEEP_CENTERPOINT_BLUE_SECOND
                : RIGHT_FIRST_SWEEP_CENTERPOINT_RED_SECOND;
    }

    public static Pose2d getLeftFirstBumpScore(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_FIRST_BUMP_SCORE_BLUE : LEFT_FIRST_BUMP_SCORE_RED;
    }

    public static Pose2d getRightFirstBumpScore(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_FIRST_BUMP_SCORE_BLUE : RIGHT_FIRST_BUMP_SCORE_RED;
    }

    public static Pose2d getLeftSecondBumpScore(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_SECOND_BUMP_SCORE_BLUE : LEFT_SECOND_BUMP_SCORE_RED;
    }

    public static Pose2d getRightSecondBumpScore(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_SECOND_BUMP_SCORE_BLUE : RIGHT_SECOND_BUMP_SCORE_RED;
    }

    public static Pose2d getLeftSecondSweepCenterpoint(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? LEFT_SECOND_SWEEP_CENTERPOINT_BLUE
                : LEFT_SECOND_SWEEP_CENTERPOINT_RED;
    }

    public static Pose2d getRightSecondSweepCenterPoint(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? RIGHT_SECOND_SWEEP_CENTERPOINT_BLUE
                : RIGHT_SECOND_SWEEP_CENTERPOINT_RED;
    }

    public static Pose2d getLeftFeedPreloadStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? LEFT_FEED_PRELOAD_STARTING_POSE_BLUE
                : LEFT_FEED_PRELOAD_STARTING_POSE_RED;
    }

    public static Pose2d getRightFeedPreloadStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? RIGHT_FEED_PRELOAD_STARTING_POSE_BLUE
                : RIGHT_FEED_PRELOAD_STARTING_POSE_RED;
    }

    public static Pose2d getLeftSweepExit(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_SWEEP_EXIT_BLUE : LEFT_SWEEP_EXIT_RED;
    }

    public static Pose2d getRightSweepExit(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_SWEEP_EXIT_BLUE : RIGHT_SWEEP_EXIT_RED;
    }

    public static Pose2d getLeftTrenchEnter(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_TRENCH_ENTER_BLUE : LEFT_TRENCH_ENTER_RED;
    }

    public static Pose2d getRightTrenchEnter(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_TRENCH_ENTER_BLUE : RIGHT_TRENCH_ENTER_RED;
    }

    public static Pose2d getLeftSecondSweepCurvepoint(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? LEFT_SECOND_SWEEP_CURVEPOINT_BLUE
                : LEFT_SECOND_SWEEP_CURVEPOINT_RED;
    }

    public static Pose2d getRightSecondSweepCurvepoint(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? RIGHT_SECOND_SWEEP_CURVEPOINT_BLUE
                : RIGHT_SECOND_SWEEP_CURVEPOINT_RED;
    }

    public static Pose2d getCenterStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? CENTER_STARTING_POSE_BLUE : CENTER_STARTING_POSE_RED;
    }

    public static Pose2d getDepotRightPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? DEPOT_RIGHT_BLUE : DEPOT_RIGHT_RED;
    }

    public static Pose2d getDepotMiddlePose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? DEPOT_MIDDLE_BLUE : DEPOT_MIDDLE_RED;
    }

    public static Pose2d getDepotLeftPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? DEPOT_LEFT_BLUE : DEPOT_LEFT_RED;
    }

    public static Pose2d getDepotScoringPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? DEPOT_SCORING_POSE_BLUE : DEPOT_SCORING_POSE_RED;
    }

    public static Pose2d getDepotTrenchEnter(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? DEPOT_TRENCH_ENTER_BLUE : DEPOT_TRENCH_ENTER_RED;
    }

    public static Pose2d getDepotTrenchEnterIntermediary(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue
                ? DEPOT_TRENCH_ENTER_INTERMEDIARY_BLUE
                : DEPOT_TRENCH_ENTER_INTERMEDIARY_RED;
    }

    public static Pose2d getDepotNZCenter(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? DEPOT_NZ_CENTER_BLUE : DEPOT_NZ_CENTER_RED;
    }

    /**
     * Check if the current alliance is blue.
     *
     * @return true if the current alliance is blue, false if red, defaults to blue if not set
     */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    /**
     * Get the 3D position of an AprilTag by its id.
     *
     * @param id the AprilTag id
     * @return the 3D pose of the tag on the field
     * @throws RuntimeException if the tag id is not recognized in the field layout
     */
    public static Pose3d getTagPose(int id) {
        return FIELD_LAYOUT.getTagPose(id).orElseThrow(() -> {
            final String message = String.format("getTagPose called for unexpected tag %d", id);
            return new RuntimeException(message);
        });
    }

    /**
     * Get the possible passing target translations for the current alliance.
     *
     * @return a collection of 2D translations where pass shots can be scored
     */
    public static Collection<Translation2d> getPossibleTranslationsToPassTo() {
        if (isBlueAlliance()) {
            return bluePassingTranslations;
        } else {
            return redPassingTranslations;
        }
    }
}
