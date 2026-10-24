package org.frc2910.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc2910.robot.RobotState;
import org.frc2910.robot.util.SubsystemToggles;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private final List<Pair<VisionIO, VisionIOInputsAutoLogged>> inputsList = new ArrayList<>();
    private final RobotState robotState;

    public VisionSubsystem(RobotState robotState, VisionIO... visionIOList) {
        for (VisionIO visionIO : visionIOList) {
            inputsList.add(Pair.of(visionIO, new VisionIOInputsAutoLogged()));
        }
        this.robotState = robotState;
    }

    @Override
    public void periodic() {
        List<RobotState.AprilTagObservation> aprilTagObservations = new ArrayList<>();
        for (Pair<VisionIO, VisionIOInputsAutoLogged> camera : inputsList) {
            var io = camera.getFirst();
            var inputs = camera.getSecond();
            io.updateInputs(inputs);
            Logger.processInputs(io.getLimelightName(), inputs);
            if (inputs.hasTargets && inputs.robotPoseBasedOffDistanceCalcAndTagLocation != null) {
                aprilTagObservations.add(new RobotState.AprilTagObservation(
                        io.getLimelightName(), inputs.tagId, inputs.robotPoseBasedOffDistanceCalcAndTagLocation));
            }
        }

        if (!SubsystemToggles.VISION.isEnabled()) {
            return;
        }
        robotState.addVisionObservation(aprilTagObservations.toArray(RobotState.AprilTagObservation[]::new));
    }
}
