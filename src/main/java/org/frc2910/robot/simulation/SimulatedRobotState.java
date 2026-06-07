package org.frc2910.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import org.frc2910.robot.RobotContainer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Helper that stores and manages simulated robot state when running in the simulator.
 *
 * <p>This class is responsible for syncing pose information between the Maple-Sim simulation
 * and robot-side components, launching simulated game pieces, and providing access to the
 * latest simulated field-to-robot pose.
 * </p>
 */
@SuppressWarnings("UnusedVariable")
public class SimulatedRobotState {

    private SwerveDriveSimulation simDrive;
    private final RobotContainer container;
    private double lastTimestamp = 0.0;

    private Pose2d fieldToRobotSimulatedTruth = Pose2d.kZero;

    public SimulatedRobotState(RobotContainer robotContainer) {
        this.container = robotContainer;
    }

    public void init() {
        this.simDrive = this.container.getDriveSubsystem().getMapleSimDrive().mapleSimDrive;
    }

    public synchronized void addFieldToRobot(Pose2d pose) {
        updateRobotPoseIfNewer(Timer.getFPGATimestamp(), pose);
    }

    public synchronized Pose2d getLatestFieldToRobot() {
        return fieldToRobotSimulatedTruth;
    }

    public void launchFuel(int count) {
        SimulatedArena.getInstance()
                .addGamePieceProjectile(new RebuiltFuelOnFly(
                        this.simDrive.getSimulatedDriveTrainPose().getTranslation(),
                        new Translation2d(-8.0 * 0.0254, 0),
                        this.simDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                        this.simDrive.getSimulatedDriveTrainPose().getRotation(),
                        Units.Meters.of(0.5),
                        Units.MetersPerSecond.of(8.0),
                        Units.Degrees.of(55)));
    }

    public synchronized void updateSim() {
        lastTimestamp = Timer.getFPGATimestamp();
    }

    private void updateRobotPoseIfNewer(double timestamp, Pose2d pose) {
        if (timestamp > lastTimestamp) {
            lastTimestamp = timestamp;
            fieldToRobotSimulatedTruth = pose;
        }
    }
}
