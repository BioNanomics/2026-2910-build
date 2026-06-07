package org.frc2910.robot.subsystems;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc2910.robot.RobotContainer;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.constants.FieldConstants;
import org.frc2910.robot.subsystems.drive.SwerveSubsystem;
import org.frc2910.robot.subsystems.hopper.HopperSubsystem;
import org.frc2910.robot.subsystems.intake.IntakeSubsystem;
import org.frc2910.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Map;

/**
 * <p>SuperStructure operates as a state machine with two layers: a {@link WantedState} set by
 * commands or auto routines, and an internal {@link CurrentState} that reflects what the robot
 * is actually doing each cycle. Each state drives various
 * subsystems to their appropriate behaviors.
 * </p>
 */
@SuppressWarnings("unused")
public class SuperStructure extends SubsystemBase {
    private final RobotContainer container;
    private final RobotState robotState;
    private final SwerveSubsystem swerveSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private boolean updateFromVisionEnabled = true;

    // Cached values updated each periodic cycle
    private Pose2d currentRobotPose = new Pose2d();
    private Translation2d targetHub = FieldConstants.BLUE_HUB.toTranslation2d();
    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private double chassisSpeedMagnitude = 0.0;
    private double distanceToHub = 0.0;
    private boolean visionObservationLatch;
    private boolean ignoreShootingTolerances = false;
    private final List<CurrentState> vaildVisionLatchingState = List.of(CurrentState.SCORING_ON_THE_MOVE);

    /**
     * Enumeration of high-level states the robot can be commanded into via the superstructure.
     */
    public enum WantedState {
        // Regular Operation SuperStates
        STOPPED,
        DEFAULT_STATE,
        HOME,
        SCORE_ON_MOVE,
        SCORE_WITHOUT_OBSERVATION,
        SCORE_KNOWN_TRANSLATION,
        PASS,
        PASS_MANUAL_ROT,

        // Auto
        AUTO_PASS,
        AUTO_SCORE,
        AUTO_SCORE_STATIONARY,

        // System characterization
        SYS_ID,
    }

    private enum CurrentState {
        // Regular Operation SuperStates
        STOPPED,
        DEFAULT_STATE,
        HOMING,
        SCORING_ON_THE_MOVE,
        SCORING_WITHOUT_OBSERVATION,
        SCORING_KNOWN_TRANSLATION,
        PASSING,
        PASSING_MANUAL_SHOT,

        // Auto
        AUTO_PASSING,
        AUTO_SCORING,
        AUTO_SCORE_STATIONARY,

        // System characterization
        SYS_ID,
    }

    public enum SwerveOverrideStates {
        OVERRIDE_SWERVE,
        DEFAULT_SWERVE
    }

    SwerveOverrideStates swerveOverrideState = SwerveOverrideStates.DEFAULT_SWERVE;

    Map<FieldConstants.KnownTranslations, Translation2d> knownTranslationsMap;

    private static final double SCORING_RADIAN_TOLERANCE = Units.degreesToRadians(4.0);
    private static final double SCORING_ON_MOVE_SPEED_TOLERANCE = 0.15;
    private static final double SOTM_TRANSLATION_SCALAR = 1.0;
    private static final double PASSING_SPEED_TOLERANCE = 10.0;
    private static final double PASSING_RADIAN_TOLERANCE = Units.degreesToRadians(15);
    private static final double POTM_TRANSLATION_SCALAR = 0.3;
    private static final double MINIMUM_DISTANCE_TO_SHOOT = 1.5;

    /**
     * tolerance in meters/second
     */
    private WantedState wantedState = WantedState.DEFAULT_STATE;

    private CurrentState currentState = CurrentState.DEFAULT_STATE;
    private IntakeSubsystem.WantedState intakeWantedState = IntakeSubsystem.WantedState.DEPLOY;
    private FieldConstants.KnownTranslations scoringKnownTranslationCurrentTranslation =
            FieldConstants.KnownTranslations.LEFT_TRENCH;
    private Pose2d poseForDriveToPoint;
    private double maxVelocityForAuto;

    /**
     * Construct the SuperStructure with references to all subsystems.
     *
     * @param container the RobotContainer containing all subsystems
     */
    public SuperStructure(RobotContainer container) {
        this.container = container;
        this.swerveSubsystem = container.getDriveSubsystem();
        this.robotState = container.getRobotState();
        this.shooterSubsystem = container.getShooterSubsystem();
        this.hopperSubsystem = container.getFeederSubsystem();
        this.intakeSubsystem = container.getIntakeSubsystem();
    }

    public void setUpdateFromVisionEnabled(final boolean updateFromVisionEnabled) {
        this.updateFromVisionEnabled = updateFromVisionEnabled;
    }

    @Override
    public void periodic() {
        // Cache values once per cycle to avoid repeated lookups
        currentRobotPose = robotState.getFieldToRobotPose();
        boolean isBlueAlliance = FieldConstants.isBlueAlliance();
        targetHub =
                isBlueAlliance ? FieldConstants.BLUE_HUB.toTranslation2d() : FieldConstants.RED_HUB.toTranslation2d();
        currentChassisSpeeds = robotState.getMeasuredFieldRelativeChassisSpeeds();
        chassisSpeedMagnitude =
                Math.hypot(currentChassisSpeeds.vxMetersPerSecond, currentChassisSpeeds.vyMetersPerSecond);
        distanceToHub = currentRobotPose.getTranslation().getDistance(targetHub);

        currentState = handleStateTransitions();

        // TODO: Improve how we relocalize for every valid vision target detected.
        // There's an issue with this approach as it doesn't compensate for the latency between
        // when the vision pose was estimated and how far the robot has moved since.
        // Fusing vision in a pose estimator would help, and we want to do that in a separate pose estimator.
        // This is just a one-time "reset" and the underlying pose estimator will clear its state and start
        // from the new "stale" pose.
        if (updateFromVisionEnabled && !robotState.getVisionObservation().isEmpty() && !visionObservationLatch) {
            swerveSubsystem.resetTranslation(robotState
                    .getVisionObservation()
                    .get(0)
                    .robotPoseFromCamera()
                    .getTranslation());
        }

        Logger.recordOutput("SuperStructure/CurrentState", currentState);
        Logger.recordOutput("SuperStructure/WantedState", wantedState);
        Logger.recordOutput("SuperStructure/IntakeWantedState", intakeWantedState);
        Logger.recordOutput("SuperStructure/DistanceToHub", distanceToHub);
        Logger.recordOutput("SuperStructure/RobotPose", currentRobotPose);
        Logger.recordOutput("SuperStructure/ScoringKnownTranslation", scoringKnownTranslationCurrentTranslation);
        Logger.recordOutput("SuperStructure/SwerveOverrideState", swerveOverrideState);
        Logger.recordOutput("SuperStructure/IgnoreShootingTolerances", ignoreShootingTolerances);

        applyStates();
    }

    private CurrentState handleStateTransitions() {
        return switch (wantedState) {
            case STOPPED -> CurrentState.STOPPED;
            case DEFAULT_STATE -> CurrentState.DEFAULT_STATE;
            case HOME -> CurrentState.HOMING;
            case SCORE_ON_MOVE -> CurrentState.SCORING_ON_THE_MOVE;
            case SCORE_WITHOUT_OBSERVATION -> CurrentState.SCORING_WITHOUT_OBSERVATION;
            case SCORE_KNOWN_TRANSLATION -> CurrentState.SCORING_KNOWN_TRANSLATION;
            case PASS_MANUAL_ROT -> CurrentState.PASSING_MANUAL_SHOT;
            case PASS -> CurrentState.PASSING;
            case AUTO_PASS -> CurrentState.AUTO_PASSING;
            case AUTO_SCORE -> CurrentState.AUTO_SCORING;
            case AUTO_SCORE_STATIONARY -> CurrentState.AUTO_SCORE_STATIONARY;
            case SYS_ID -> CurrentState.SYS_ID;
        };
    }

    private void applyStates() {
        if (!vaildVisionLatchingState.contains(currentState)) {
            visionObservationLatch = false;
        }
        switch (currentState) {
            case STOPPED -> stopped();
            case DEFAULT_STATE -> defaultState();
            case HOMING -> homing();
            case SCORING_ON_THE_MOVE -> scoringOnTheMove(true);
            case SCORING_WITHOUT_OBSERVATION -> scoringOnTheMove(false);
            case SCORING_KNOWN_TRANSLATION -> scoringKnownTranslation();
            case PASSING -> passing();
            case PASSING_MANUAL_SHOT -> passingManualRot();
            case AUTO_PASSING -> autoPassing();
            case AUTO_SCORING -> autoScoring();
            case AUTO_SCORE_STATIONARY -> autoScoreStationary();
            case SYS_ID -> {}
        }
    }

    private void autoScoreStationary() {
        var targetRotation = targetHub.minus(currentRobotPose.getTranslation()).getAngle();
        var rpm = container.getShooterInterpolation().getInterpolatedRPM(distanceToHub);
        var angle = container.getShooterInterpolation().getInterpolatedHoodAngle(distanceToHub);
        setShooterAndFeederShotConditions(rpm, angle);
        setIntakeShootingState();

        Logger.recordOutput("SuperStructure/ScoringAutoPreload/RPM", rpm);
        Logger.recordOutput("SuperStructure/ScoringAutoPreload/Angle", angle);
        Logger.recordOutput("SuperStructure/ScoringAutoPreload/Distance", distanceToHub);

        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setSwerveToCalculatedState(targetRotation, SOTM_TRANSLATION_SCALAR);

        if (shooterSubsystem.isAtShotConditions() && swerveSubsystem.isAtDesiredRotation()) {
            hopperSubsystem.setWantedState(HopperSubsystem.WantedState.SHOOT);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.RETRACT);
        } else {
            hopperSubsystem.setWantedState(HopperSubsystem.WantedState.DEFAULT);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY);
        }
    }

    // SuperState Functions:
    private void stopped() {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY);
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.OFF);
        hopperSubsystem.setWantedState(HopperSubsystem.WantedState.DEFAULT);
        swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.IDLE);
    }

    private void defaultState() {
        intakeSubsystem.setWantedState(intakeWantedState);
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setFeederAndIntakeToCalculatedState(true);
        var rpm = container.getShooterInterpolation().getInterpolatedRPM(distanceToHub);
        var angle = 0.0;
        // Set the values
        setShooterAndFeederShotConditions(rpm, angle);

        Logger.recordOutput("SuperStructure/Default/RPM", rpm);
        Logger.recordOutput("SuperStructure/Default/Angle", angle);
        Logger.recordOutput("SuperStructure/Default/Distance", distanceToHub);

        // Default: let the driver control it.
        if (!swerveSubsystem.isFollowingPath()
                && swerveSubsystem.getWantedState() != SwerveSubsystem.WantedState.SEEK
                && swerveSubsystem.getWantedState() != SwerveSubsystem.WantedState.SET_STATOR_CURRENT_LIMIT
                && swerveSubsystem.getWantedState() != SwerveSubsystem.WantedState.ROTATION_LOCK) {
            setSwerveToCalculatedState();
        }
    }

    private void homing() {
        if (DriverStation.isEnabled()) {
            shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.HOME);
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        } else {
            shooterSubsystem.manualHome();
            intakeSubsystem.manualHome();
        }
    }

    public boolean isHomed() {
        return intakeSubsystem.isHomed() && shooterSubsystem.isHoodHomed();
    }

    private void scoringOnTheMove(boolean requireVisionObservation) {
        var translation = currentRobotPose.getTranslation();
        var targetRotation = targetHub.minus(translation).getAngle();

        Logger.recordOutput(
                "SuperStructure/ScoringOnTheMove/ExtrapolatedTranslation", new Pose2d(translation, targetRotation));

        var distance = getDistanceToCenterOfHub(translation);
        var rpm = container.getShooterInterpolation().getInterpolatedRPM(distance)
                * container.getOperatorDashboard().getRPMMultiplier();
        var angle = container.getShooterInterpolation().getInterpolatedHoodAngle(distance);
        setShooterAndFeederShotConditions(rpm, angle);

        Logger.recordOutput("SuperStructure/ScoringOnTheMove/RPM", rpm);
        Logger.recordOutput("SuperStructure/ScoringOnTheMove/Angle", angle);
        Logger.recordOutput("SuperStructure/ScoringOnTheMove/Distance", distance);

        setSwerveToCalculatedState(targetRotation, SOTM_TRANSLATION_SCALAR);
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setFeederAndIntakeToCalculatedState(requireVisionObservation);
    }

    private void scoringKnownTranslation() {
        // Assuming all mechanical functions work but not relocalizing to the correct position
        // Function sets translation to selected knownTranslation and shoots as if robot were there

        Translation2d knownTranslation = knownTranslationsMap.get(scoringKnownTranslationCurrentTranslation);

        swerveSubsystem.resetTranslation(knownTranslation);

        var targetRotation = targetHub.minus(knownTranslation).getAngle();
        setSwerveToCalculatedState(targetRotation, SOTM_TRANSLATION_SCALAR);

        var distance = getDistanceToCenterOfHub(knownTranslation);
        var rpm = container.getShooterInterpolation().getInterpolatedRPM(distance)
                * container.getOperatorDashboard().getRPMMultiplier();
        var angle = container.getShooterInterpolation().getInterpolatedHoodAngle(distance);
        // Set the values
        setShooterAndFeederShotConditions(rpm, angle);

        Logger.recordOutput("SuperStructure/ScoringKnownTranslation/RPM", rpm);
        Logger.recordOutput("SuperStructure/ScoringKnownTranslation/Angle", angle);
        Logger.recordOutput("SuperStructure/ScoringKnownTranslation/Distance", distance);

        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setFeederAndIntakeToCalculatedState(false);
    }

    private void autoScoring() {
        var translation = currentRobotPose.getTranslation();
        var targetRotation = targetHub.minus(translation).getAngle();

        var distance = getDistanceToCenterOfHub(translation);
        var rpm = container.getShooterInterpolation().getInterpolatedRPM(distance)
                * container.getOperatorDashboard().getRPMMultiplier();
        var angle = container.getShooterInterpolation().getInterpolatedHoodAngle(distance);
        // Set the values
        setShooterAndFeederShotConditions(rpm, angle);

        Logger.recordOutput("SuperStructure/AutoScoring/RPM", rpm);
        Logger.recordOutput("SuperStructure/AutoScoring/Angle", angle);
        Logger.recordOutput("SuperStructure/AutoScoring/Distance", distance);

        var poseToDriveTo = new Pose2d(poseForDriveToPoint.getX(), poseForDriveToPoint.getY(), targetRotation);

        // Used for hub targeting only in drive to point
        // TODO: handle automatically if in drive to point
        swerveSubsystem.setDesiredPoseForDriveToPoint(poseToDriveTo, Units.feetToMeters(maxVelocityForAuto));
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);

        if (isReadyToScoreAuto()) {
            hopperSubsystem.setWantedState(HopperSubsystem.WantedState.SHOOT);
            setIntakeShootingState();
        } else {
            hopperSubsystem.setWantedState(HopperSubsystem.WantedState.DEFAULT);
            intakeSubsystem.setWantedState(intakeWantedState);
        }
    }

    private void passing() {
        Translation2d translationToPassTo;
        translationToPassTo = getTranslationToPassTo();

        var extrapolatedTranslation = getExtrapolatedTranslation(container
                .getPassingInterpolation()
                .getInterpolatedTimeOfFlight(getDistanceToTranslation(translationToPassTo)));

        var targetRotation = translationToPassTo.minus(extrapolatedTranslation).getAngle();

        Logger.recordOutput(
                "SuperStructure/Passing/ExtrapolatedTranslation", new Pose2d(extrapolatedTranslation, targetRotation));

        double rpm;
        double angle;
        double distance = getDistanceToTranslation(extrapolatedTranslation, translationToPassTo);

        rpm = container.getPassingInterpolation().getInterpolatedRPM(distance);
        angle = container.getPassingInterpolation().getInterpolatedAngle(distance);

        rpm *= container.getOperatorDashboard().getRPMPassMultiplier();

        // Set the values
        setShooterAndFeederShotConditions(rpm, angle);
        Logger.recordOutput("SuperStructure/Passing/RPM", rpm);
        Logger.recordOutput("SuperStructure/Passing/Angle", angle);
        Logger.recordOutput("SuperStructure/Passing/Distance", distance);

        setSwerveToCalculatedState(targetRotation, POTM_TRANSLATION_SCALAR);
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setFeederAndIntakeToCalculatedState(true);
        // To not accidentally trigger isReadyToScore() instead of isReadyToPass() even though we don't require
        // observation for passing
    }

    private void passingManualRot() {
        // Assume hood and flywheels are roughly correct but rotation is not
        Translation2d translationToPassTo;
        translationToPassTo = getTranslationToPassTo();

        var extrapolatedTranslation = getExtrapolatedTranslation(container
                .getPassingInterpolation()
                .getInterpolatedTimeOfFlight(getDistanceToTranslation(translationToPassTo)));

        double rpm;
        double angle;
        double distance = getDistanceToTranslation(extrapolatedTranslation, translationToPassTo);

        rpm = container.getPassingInterpolation().getInterpolatedRPM(distance);
        angle = container.getPassingInterpolation().getInterpolatedAngle(distance);

        rpm *= container.getOperatorDashboard().getRPMPassMultiplier();

        // Set the values
        setShooterAndFeederShotConditions(rpm, angle);
        Logger.recordOutput("SuperStructure/ManualPass/RPM", rpm);
        Logger.recordOutput("SuperStructure/ManualPass/Angle", angle);
        Logger.recordOutput("SuperStructure/ManualPass/Distance", distance);

        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setFeederAndIntakeToCalculatedState(true);
        // To not accidentally trigger isReadyToScore() instead of isReadyToPass() even though we don't require
        // observation for passing
    }

    private void autoPassing() {
        var translationToPassTo = getTranslationToPassTo();
        var extrapolatedTranslation = getExtrapolatedTranslation(container
                .getPassingInterpolation()
                .getInterpolatedTimeOfFlight(getDistanceToTranslation(translationToPassTo)));

        var targetRotation = translationToPassTo.minus(extrapolatedTranslation).getAngle();
        var poseToDriveTo = new Pose2d(poseForDriveToPoint.getX(), poseForDriveToPoint.getY(), targetRotation);

        var distance = getDistanceToTranslation(extrapolatedTranslation, translationToPassTo);
        var rpm = container.getPassingInterpolation().getInterpolatedRPM(distance)
                * container.getOperatorDashboard().getRPMPassMultiplier();
        var angle = container.getPassingInterpolation().getInterpolatedAngle(distance);
        // Set the values
        setShooterAndFeederShotConditions(rpm, angle);

        Logger.recordOutput("SuperStructure/AutoPass/RPM", rpm);
        Logger.recordOutput("SuperStructure/AutoPass/Angle", angle);
        Logger.recordOutput("SuperStructure/AutoPass/Distance", distance);

        swerveSubsystem.setDesiredPoseForDriveToPoint(poseToDriveTo, maxVelocityForAuto);
        shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.RUN);
        setFeederAndIntakeToCalculatedState(true);
        // To not accidentally trigger isReadyToScore() instead of isReadyToPass() even though we don't require
        // observation for passing
    }

    public void driveToPoint(Pose2d point, double maxVelocity) {
        // NOTE: we don't set a "wanted state" here, since we just want to
        // drive somewhere, not necessarily score or pass after driving there.
        // We just set the swerve wanted state to drive to point and let the
        // rest of the code handle it.
        poseForDriveToPoint = point;
        setIntakeWantedState(IntakeSubsystem.WantedState.DEPLOY);
        swerveSubsystem.setDesiredPoseForDriveToPoint(point, maxVelocity);
    }

    public void driveToPoint(Pose2d point, double maxVelocity, double toleranceInInches) {
        // NOTE: we don't set a "wanted state" here, since we just want to
        // drive somewhere, not necessarily score or pass after driving there.
        // We just set the swerve wanted state to drive to point and let the
        // rest of the code handle it.
        poseForDriveToPoint = point;
        setIntakeWantedState(IntakeSubsystem.WantedState.DEPLOY);
        swerveSubsystem.setDesiredPoseForDriveToPoint(point, maxVelocity, toleranceInInches);
    }

    public void driveToPointAndIntake(Pose2d point, double maxVelocity) {
        poseForDriveToPoint = point;
        setIntakeWantedState(IntakeSubsystem.WantedState.INTAKE);
        swerveSubsystem.setDesiredPoseForDriveToPoint(point, maxVelocity);
    }

    public void driveToPointAndIntake(Pose2d point, double maxVelocity, double toleranceInInches) {
        poseForDriveToPoint = point;
        setIntakeWantedState(IntakeSubsystem.WantedState.INTAKE);
        swerveSubsystem.setDesiredPoseForDriveToPoint(point, maxVelocity, toleranceInInches);
    }

    public void followPathPlannerTrajectory(PathPlannerTrajectory desiredTrajectory) {
        followPathPlannerTrajectory(desiredTrajectory, WantedState.DEFAULT_STATE);
    }

    public void followPathPlannerTrajectory(PathPlannerTrajectory desiredTrajectory, WantedState wantedState) {
        this.wantedState = wantedState;
        swerveSubsystem.setTrajectoryToFollow(desiredTrajectory);
    }

    public void followTrajectoryAndIntakeWithSOTM(PathPlannerTrajectory trajectory) {
        poseForDriveToPoint = trajectory.getEndState().pose;
        setIntakeWantedState(IntakeSubsystem.WantedState.INTAKE);
        followPathPlannerTrajectory(trajectory, WantedState.AUTO_SCORE);
    }

    public void followTrajectoryAndIntakeWithPOTM(PathPlannerTrajectory trajectory) {
        poseForDriveToPoint = trajectory.getEndState().pose;
        setIntakeWantedState(IntakeSubsystem.WantedState.INTAKE);
        followPathPlannerTrajectory(trajectory, WantedState.AUTO_PASS);
    }

    public void followTrajectoryAndIntake(PathPlannerTrajectory trajectory) {
        setIntakeWantedState(IntakeSubsystem.WantedState.INTAKE);
        followPathPlannerTrajectory(trajectory);
    }

    // Subsystem calculated state functions
    private void setFeederAndIntakeToCalculatedState(boolean requireVisionObservation) {
        var isReadyToScoreResult = isReadyToScore(requireVisionObservation);
        Logger.recordOutput("SuperStructure/ShotConditions/isReadyToScore", isReadyToScoreResult);
        Logger.recordOutput("SuperStructure/PassConditions/isReadyToPass", isReadyToPass());
        if (isReadyToScoreResult || isReadyToPass()) {
            hopperSubsystem.setWantedState(HopperSubsystem.WantedState.SHOOT);
            setIntakeShootingState();
        } else {
            hopperSubsystem.setWantedState(HopperSubsystem.WantedState.DEFAULT);
            intakeSubsystem.setWantedState(intakeWantedState);
        }
    }

    private void setSwerveToCalculatedState() {
        if (swerveOverrideState == SwerveOverrideStates.OVERRIDE_SWERVE) {
            swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.ANTI_DEFENSE_WHEEL_LOCK);
        } else {
            swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.TELEOP);
        }
    }

    private void setSwerveToCalculatedState(Rotation2d targetRotation, double translationScalar) {
        if (swerveOverrideState == SwerveOverrideStates.OVERRIDE_SWERVE) {
            swerveSubsystem.setWantedState(SwerveSubsystem.WantedState.ANTI_DEFENSE_WHEEL_LOCK);
        } else {
            swerveSubsystem.setTargetRotationWithLimitedTranslation(targetRotation, translationScalar);
        }
    }

    private Translation2d getTranslationToPassTo() {
        return currentRobotPose.getTranslation().nearest(FieldConstants.getPossibleTranslationsToPassTo());
    }

    private double getDistanceToTranslation(
            Translation2d translation2dToMeasureFrom, Translation2d translation2dToMeasureTo) {
        return translation2dToMeasureFrom.getDistance(translation2dToMeasureTo);
    }

    private double getDistanceToTranslation(Translation2d translationToMeasureTo) {
        return currentRobotPose.getTranslation().getDistance(translationToMeasureTo);
    }

    private double getDistanceToCenterOfHub(Translation2d translationToMeasureDistanceFrom) {
        return translationToMeasureDistanceFrom.getDistance(targetHub);
    }

    private boolean isReadyToScore(boolean requireVisionObservation) {
        Logger.recordOutput(
                "SuperStructure/ShotConditions/serpentineShotConditionsMet", shooterSubsystem.isAtShotConditions());
        Logger.recordOutput(
                "SuperStructure/ShotConditions/isAtDesiredRotation",
                swerveSubsystem.isAtDesiredRotation(SCORING_RADIAN_TOLERANCE));
        Logger.recordOutput(
                "SuperStructure/ShotConditions/hasObservation",
                !robotState.getVisionObservation().isEmpty());
        Logger.recordOutput(
                "SuperStructure/ShotConditions/withinSpeedTolerance",
                chassisSpeedMagnitude <= SCORING_ON_MOVE_SPEED_TOLERANCE);
        Logger.recordOutput("SuperStructure/ShotConditions/chassisSpeedMagnitude", chassisSpeedMagnitude);
        if ((currentState == CurrentState.SCORING_ON_THE_MOVE
                        || currentState == CurrentState.AUTO_SCORING
                        || currentState == CurrentState.SCORING_KNOWN_TRANSLATION
                        || currentState == CurrentState.SCORING_WITHOUT_OBSERVATION)
                && ((shooterSubsystem.isAtShotConditions()
                                && (getDistanceToCenterOfHub(currentRobotPose.getTranslation())
                                        >= MINIMUM_DISTANCE_TO_SHOOT)
                                && swerveSubsystem.isAtDesiredRotation(SCORING_RADIAN_TOLERANCE)
                                && chassisSpeedMagnitude <= SCORING_ON_MOVE_SPEED_TOLERANCE)
                        || ignoreShootingTolerances)) {
            return !requireVisionObservation || hasSeenAprilTagForShot() || ignoreShootingTolerances;
        } else {
            visionObservationLatch = false;
            return false;
        }
    }

    private boolean hasSeenAprilTagForShot() {
        return visionObservationLatch |= !robotState.getVisionObservation().isEmpty();
    }

    private boolean isReadyToScoreAuto() {
        Logger.recordOutput(
                "SuperStructure/ShotConditions/serpentineShotConditionsMet", shooterSubsystem.isAtShotConditions());
        return (currentState == CurrentState.AUTO_SCORING)
                && shooterSubsystem.isAtShotConditions()
                && swerveSubsystem.isAtDesiredRotation();
    }

    private boolean isReadyToPass() {
        var passingSpeed = currentChassisSpeeds.vyMetersPerSecond;
        var isAtShotConditionsResult = shooterSubsystem.isAtShotConditions();
        var isAtDesiredRotationResult = swerveSubsystem.isAtDesiredRotation(PASSING_RADIAN_TOLERANCE);
        Logger.recordOutput("SuperStructure/PassConditions/serpentineShotConditionsMet", isAtShotConditionsResult);
        Logger.recordOutput("SuperStructure/PassConditions/isAtDesiredRotation", isAtDesiredRotationResult);
        Logger.recordOutput(
                "SuperStructure/PassConditions/withinSpeedTolerance", chassisSpeedMagnitude <= passingSpeed);
        Logger.recordOutput("SuperStructure/PassConditions/SpeedMagnitude", passingSpeed);
        return (((currentState == CurrentState.PASSING || currentState == CurrentState.AUTO_PASSING)
                        && ((isAtShotConditionsResult
                                        && isAtDesiredRotationResult
                                        && Math.abs(passingSpeed) <= PASSING_SPEED_TOLERANCE)
                                || ignoreShootingTolerances))
                || (currentState == CurrentState.PASSING_MANUAL_SHOT
                        && ((isAtShotConditionsResult && Math.abs(passingSpeed) <= PASSING_SPEED_TOLERANCE)
                                || ignoreShootingTolerances)));
    }

    private Translation2d getExtrapolatedTranslation(double time) {
        // NOTE: The method for extrapolating the translation of the robot after a certain
        // amount of time has passed is to take the current velocity of the robot and multiply
        // it by the time and assumes a straight line path.
        // It doesn't take into account robot acceleration/deceleration nor path curvature.
        // We can do better if needed.  @see EulerRollout class.
        Translation2d extrapolatedTranslation = new Translation2d(
                (currentChassisSpeeds.vxMetersPerSecond * time), (currentChassisSpeeds.vyMetersPerSecond * time));
        return currentRobotPose.getTranslation().plus(extrapolatedTranslation);
    }

    // SuperStructure setters:
    /**
     * Sets the wanted super state directly.
     */
    public void setWantedStateCommand(WantedState superState) {
        this.wantedState = superState;
    }

    public void setScoringKnownTranslation(FieldConstants.KnownTranslations translation) {
        this.scoringKnownTranslationCurrentTranslation = translation;
    }

    public void setSwerveOverrideState(SwerveOverrideStates swerveOverrideState) {
        this.swerveOverrideState = swerveOverrideState;
    }

    public void setIntakeWantedState(IntakeSubsystem.WantedState wantedState) {
        intakeWantedState = wantedState;
    }

    private void setIntakeShootingState() {
        if (intakeWantedState != IntakeSubsystem.WantedState.INTAKE) {
            intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.RETRACT);
        } else {
            intakeSubsystem.setWantedState(intakeWantedState);
        }
    }

    private void setShooterAndFeederShotConditions(double RPM, double hoodAngle) {
        shooterSubsystem.setShotConditions(RPM, hoodAngle);
    }

    public void setKnownTranslationsMapForAlliance(DriverStation.Alliance alliance) {
        if (alliance == DriverStation.Alliance.Blue) {
            knownTranslationsMap = FieldConstants.blueKnownTranslationsMap;
        } else {
            knownTranslationsMap = FieldConstants.redKnownTranslationsMap;
        }
        Logger.recordOutput(
                "SuperStructure/LEFT_TRENCH", knownTranslationsMap.get(FieldConstants.KnownTranslations.LEFT_TRENCH));
        Logger.recordOutput(
                "SuperStructure/RIGHT_TRENCH", knownTranslationsMap.get(FieldConstants.KnownTranslations.RIGHT_TRENCH));
        Logger.recordOutput(
                "SuperStructure/LEFT_UPRIGHT", knownTranslationsMap.get(FieldConstants.KnownTranslations.LEFT_UPRIGHT));
        Logger.recordOutput(
                "SuperStructure/RIGHT_UPRIGHT",
                knownTranslationsMap.get(FieldConstants.KnownTranslations.RIGHT_UPRIGHT));
    }

    public void setIgnoreShootingTolerances(boolean ignoreShootingTolerances) {
        this.ignoreShootingTolerances = ignoreShootingTolerances;
    }
}
