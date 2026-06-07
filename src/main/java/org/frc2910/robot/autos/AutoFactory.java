package org.frc2910.robot.autos;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.frc2910.robot.RobotContainer;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.subsystems.SuperStructure;
import org.frc2910.robot.subsystems.drive.SwerveSubsystem;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
class AutoFactory {
    private static final double DTP_FAST_VELOCITY_METERS = Units.feetToMeters(15.0);
    private static final double DTP_MEDIUM_VELOCITY_METERS = Units.feetToMeters(11.0);
    private static final double DTP_SLOW_VELOCITY_METERS = Units.feetToMeters(5.0);

    private final DriverStation.Alliance alliance;

    private final RobotContainer robotContainer;

    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private final SwerveSubsystem swerveSubsystem;

    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private final RobotConfig pathPlannerConfig;

    private final Rotation2d[] DEFAULT_MODULE_POSITIONS =
            new Rotation2d[] {Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero};
    private final Rotation2d[] MODULE_POSITIONS_FOR_ROTATION = new Rotation2d[] {
        Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(45),
        Rotation2d.fromDegrees(-135), Rotation2d.fromDegrees(-45)
    };

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;
        this.swerveSubsystem = robotContainer.getDriveSubsystem();

        try {
            this.pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
        }
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Pair<Pose2d, Command> createIdleCommand() {
        return Pair.of(FieldConstants.getLeftFeedPreloadStartingPose(alliance), IDLE_COMMAND);
    }

    Pair<Pose2d, Command> createTestAuto() {
        return Pair.of(
                FieldConstants.getLeftFeedPreloadStartingPose(alliance),
                driveToPoint(
                        FieldConstants.getLeftFirstSweepEntry(alliance), Units.feetToMeters(DTP_SLOW_VELOCITY_METERS)));
    }

    // REBLITZ AUTOS:
    Pair<Pose2d, Command> createLeftNeutralZoneAuto() {
        return Pair.of(
                FieldConstants.getLeftFeedPreloadStartingPose(alliance),
                Commands.sequence(
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepEntry(alliance), DTP_FAST_VELOCITY_METERS, 3.5),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftFirstSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftFirstSweepCenterpoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.5),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepCenterpoint(alliance), DTP_FAST_VELOCITY_METERS, 3.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepCenterpointSecond(alliance),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPoint(FieldConstants.getLeftSweepExit(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getLeftFirstBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS, 1.0),
                        driveToPoint(FieldConstants.getLeftSecondBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        driveToPointWhileIntaking(
                                        FieldConstants.getLeftTrenchEnter(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.75)
                                .raceWith(Commands.waitSeconds(2.0)),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftTrenchEnter(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.0),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftSecondSweepCenterpoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftSecondSweepCenterpoint(alliance),
                                DTP_MEDIUM_VELOCITY_METERS,
                                1.5),
                        driveToPointWhileIntaking(FieldConstants.getLeftSecondSweepCurvepoint(alliance), 0.01)
                                .raceWith(Commands.waitSeconds(0.5)),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftSecondSweepCurvepoint(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getLeftSweepExit(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getLeftFirstBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS, 1.0),
                        driveToPoint(FieldConstants.getLeftSecondBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        driveToPointWhileIntaking(
                                        FieldConstants.getLeftTrenchEnter(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.75)
                                .raceWith(Commands.waitSeconds(2.0)),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftTrenchEnter(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.0),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftSecondSweepCenterpoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepCenterpoint(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE))));
    }

    Pair<Pose2d, Command> createRightNeutralZoneAuto() {
        return Pair.of(
                FieldConstants.getRightFeedPreloadStartingPose(alliance),
                Commands.sequence(
                        driveToPointWhileIntaking(
                                FieldConstants.getRightFirstSweepEntry(alliance), DTP_FAST_VELOCITY_METERS, 2.5),
                        driveToPointWhileIntaking(
                                FieldConstants.getRightFirstSweepCenterpoint(alliance), DTP_FAST_VELOCITY_METERS, 2.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getRightFirstSweepCenterpointSecond(alliance),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPoint(FieldConstants.getRightSweepExit(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getRightFirstBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS, 1.0),
                        driveToPoint(FieldConstants.getRightSecondBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        driveToPointWhileIntaking(
                                        FieldConstants.getRightTrenchEnter(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.75)
                                .raceWith(Commands.waitSeconds(2.0)),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getRightSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getRightTrenchEnter(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.0),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getRightSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getRightSecondSweepCenterPoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getRightSecondSweepCenterPoint(alliance),
                                DTP_MEDIUM_VELOCITY_METERS,
                                1.5),
                        driveToPointWhileIntaking(FieldConstants.getRightSecondSweepCurvepoint(alliance), 0.01)
                                .raceWith(Commands.waitSeconds(0.5)),
                        driveToPointWhileIntaking(
                                FieldConstants.getRightSecondSweepCurvepoint(alliance),
                                DTP_MEDIUM_VELOCITY_METERS,
                                0.5),
                        driveToPoint(FieldConstants.getRightSweepExit(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getRightFirstBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS, 1.0),
                        driveToPoint(FieldConstants.getRightSecondBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        driveToPointWhileIntaking(
                                        FieldConstants.getRightTrenchEnter(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.75)
                                .raceWith(Commands.waitSeconds(2.0)),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getRightSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getRightTrenchEnter(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.0),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getRightSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getRightSecondSweepCenterPoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getRightFirstSweepCenterpoint(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE))));
    }

    Pair<Pose2d, Command> createLeftNeutralZoneToDepotAuto() {
        return Pair.of(
                FieldConstants.getLeftFeedPreloadStartingPose(alliance),
                Commands.sequence(
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepEntry(alliance), DTP_FAST_VELOCITY_METERS, 3.5),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftFirstSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftFirstSweepCenterpoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.5),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepCenterpoint(alliance), DTP_FAST_VELOCITY_METERS, 2.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftFirstSweepCenterpointSecond(alliance),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPoint(FieldConstants.getLeftSweepExit(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getLeftFirstBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS, 1.0),
                        driveToPoint(FieldConstants.getLeftSecondBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        driveToPointWhileIntaking(
                                        FieldConstants.getLeftTrenchEnter(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.75)
                                .raceWith(Commands.waitSeconds(2.0)),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftTrenchEnter(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                2.0),
                        driveToPointWhileIntaking(
                                new Pose2d(
                                        FieldConstants.getLeftSecondSweepEntry(alliance)
                                                .getTranslation(),
                                        FieldConstants.getLeftSecondSweepCenterpoint(alliance)
                                                .getRotation()),
                                DTP_FAST_VELOCITY_METERS,
                                1.0),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftSecondSweepCenterpoint(alliance),
                                DTP_MEDIUM_VELOCITY_METERS,
                                1.5),
                        driveToPointWhileIntaking(FieldConstants.getLeftSecondSweepCurvepoint(alliance), 0.01)
                                .raceWith(Commands.waitSeconds(0.5)),
                        driveToPointWhileIntaking(
                                FieldConstants.getLeftSecondSweepCurvepoint(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getLeftSweepExit(alliance), DTP_MEDIUM_VELOCITY_METERS, 0.5),
                        driveToPoint(FieldConstants.getLeftFirstBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS, 1.0),
                        driveToPoint(FieldConstants.getLeftSecondBumpScore(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        driveToPointWhileIntaking(
                                FieldConstants.getDepotRightPose(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        scoreStationary(),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE))));
    }

    Pair<Pose2d, Command> createDepotAuto() {
        return Pair.of(
                FieldConstants.getCenterStartingPose(alliance),
                Commands.sequence(
                        Commands.parallel(
                                Commands.waitSeconds(15.0),
                                Commands.sequence(
                                        driveToPointWhileIntaking(
                                                FieldConstants.getDepotRightPose(alliance), DTP_SLOW_VELOCITY_METERS),
                                        Commands.waitSeconds(1.0),
                                        driveToPointWhileIntaking(
                                                FieldConstants.getDepotMiddlePose(alliance),
                                                DTP_MEDIUM_VELOCITY_METERS),
                                        driveToPointWhileIntaking(
                                                FieldConstants.getDepotLeftPose(alliance), DTP_SLOW_VELOCITY_METERS),
                                        Commands.waitSeconds(1.0),
                                        driveToPointWhileIntaking(
                                                FieldConstants.getDepotScoringPose(alliance),
                                                DTP_MEDIUM_VELOCITY_METERS),
                                        scoreStationary(),
                                        driveToPointWhileIntaking(
                                                FieldConstants.getDepotTrenchEnterIntermediary(alliance),
                                                DTP_SLOW_VELOCITY_METERS),
                                        driveToPointWhileIntaking(
                                                FieldConstants.getDepotTrenchEnter(alliance),
                                                DTP_SLOW_VELOCITY_METERS))),
                        driveToPointWhileIntaking(
                                FieldConstants.getDepotNZCenter(alliance), DTP_MEDIUM_VELOCITY_METERS),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE))));
    }

    Command scoreStationary() {
        return Commands.runOnce(() -> robotContainer
                        .getSuperStructure()
                        .setWantedStateCommand(SuperStructure.WantedState.AUTO_SCORE_STATIONARY))
                .alongWith(Commands.race(
                        Commands.waitSeconds(4.0),
                        Commands.sequence(
                                Commands.waitUntil(robotContainer.getIntakeSubsystem()::isFullyRetracted),
                                Commands.waitSeconds(0.6))));
    }

    Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return Commands.parallel(
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE)),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .driveToPoint(point, maxVelocityOutputForDriveToPoint)),
                        Commands.waitUntil(
                                () -> robotContainer.getDriveSubsystem().isAtDriveToPointSetpoint()))
                .andThen(Commands.runOnce(
                        () -> robotContainer.getDriveSubsystem().setWantedState(SwerveSubsystem.WantedState.IDLE)));
    }

    Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint, double toleranceInMeters) {
        return Commands.parallel(
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE)),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .driveToPoint(point, maxVelocityOutputForDriveToPoint, toleranceInMeters)),
                        Commands.waitUntil(
                                () -> robotContainer.getDriveSubsystem().isAtDriveToPointSetpoint()))
                .andThen(Commands.runOnce(
                        () -> robotContainer.getDriveSubsystem().setWantedState(SwerveSubsystem.WantedState.IDLE)));
    }

    Command driveToPointWhileIntaking(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return Commands.parallel(
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE)),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .driveToPointAndIntake(point, maxVelocityOutputForDriveToPoint)),
                        Commands.waitUntil(
                                () -> robotContainer.getDriveSubsystem().isAtDriveToPointSetpoint()))
                .andThen(Commands.runOnce(
                        () -> robotContainer.getDriveSubsystem().setWantedState(SwerveSubsystem.WantedState.IDLE)));
    }

    Command driveToPointWhileIntaking(Pose2d point, double maxVelocityOutputForDriveToPoint, double toleranceInMeters) {
        return Commands.parallel(
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE)),
                        Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .driveToPointAndIntake(point, maxVelocityOutputForDriveToPoint, toleranceInMeters)),
                        Commands.waitUntil(
                                () -> robotContainer.getDriveSubsystem().isAtDriveToPointSetpoint()))
                .andThen(Commands.runOnce(
                        () -> robotContainer.getDriveSubsystem().setWantedState(SwerveSubsystem.WantedState.IDLE)));
    }

    // region System characterization
    Pair<Pose2d, Command> sysIdTranslation() {
        return createSysIdCommand(SwerveSubsystem.WantedState.SYS_ID_TRANSLATION);
    }

    Pair<Pose2d, Command> sysIdRotation() {
        return createSysIdCommand(SwerveSubsystem.WantedState.SYS_ID_ROTATION);
    }

    Pair<Pose2d, Command> sysIdSteer() {
        return createSysIdCommand(SwerveSubsystem.WantedState.SYS_ID_STEER);
    }

    private Pair<Pose2d, Command> createSysIdCommand(final SwerveSubsystem.WantedState driveWantedState) {
        final SysIdRoutine routine = robotContainer.getDriveSubsystem().getSysIdRoutine(driveWantedState);

        return Pair.of(
                Pose2d.kZero,
                Commands.sequence(
                        createPrepareForSysIdCommand(driveWantedState),
                        createSysIdRoutineSequenceCommand(routine),
                        createFinishSysIdCommand()));
    }

    private Command createSetModulePositionsCommand(final Rotation2d[] modulePositions) {
        return Commands.sequence(
                Commands.runOnce(() -> robotContainer.getDriveSubsystem().setModulePositions(modulePositions)),
                Commands.waitUntil(() -> !robotContainer.getDriveSubsystem().isSettingModulePositions()));
    }

    private Command createPrepareForSysIdCommand(final SwerveSubsystem.WantedState driveWantedState) {
        final Command resetCommand =
                switch (driveWantedState) {
                    case SYS_ID_TRANSLATION, SYS_ID_STEER -> createSetModulePositionsCommand(DEFAULT_MODULE_POSITIONS);
                    case SYS_ID_ROTATION -> createSetModulePositionsCommand(MODULE_POSITIONS_FOR_ROTATION);
                    default -> Commands.none();
                };

        return Commands.sequence(
                Commands.parallel(
                        Commands.runOnce(
                                () -> robotContainer.getSuperStructure().setUpdateFromVisionEnabled(false)),
                        resetCommand,
                        Commands.sequence(
                                Commands.runOnce(() -> SignalLogger.enableAutoLogging(true)),
                                Commands.runOnce(SignalLogger::start))),
                Commands.runOnce(() -> robotContainer
                                .getSuperStructure()
                                .setWantedStateCommand(SuperStructure.WantedState.SYS_ID))
                        .andThen(Commands.runOnce(
                                () -> robotContainer.getDriveSubsystem().setWantedState(driveWantedState))));
    }

    private Command createSysIdRoutineSequenceCommand(final SysIdRoutine routine) {
        return Commands.sequence(
                routine.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0), // 1 s
                routine.quasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1.0), // 1 s
                routine.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0), // 1 s
                routine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    private Command createFinishSysIdCommand() {
        return Commands.sequence(
                Commands.runOnce(
                        () -> robotContainer.getDriveSubsystem().setWantedState(SwerveSubsystem.WantedState.IDLE)),
                Commands.runOnce(() -> robotContainer
                        .getSuperStructure()
                        .setWantedStateCommand(SuperStructure.WantedState.DEFAULT_STATE)),
                Commands.parallel(
                        Commands.sequence(
                                Commands.runOnce(SignalLogger::stop),
                                Commands.runOnce(() -> SignalLogger.enableAutoLogging(false))),
                        Commands.runOnce(
                                () -> robotContainer.getSuperStructure().setUpdateFromVisionEnabled(true))));
    }
}
