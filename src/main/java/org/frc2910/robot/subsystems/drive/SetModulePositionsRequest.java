package org.frc2910.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.stream.IntStream;

/**
 * A {@link SwerveRequest} to move the swerve modules to specific positions.
 * This class honors absolute positions, even if it means rotating the model
 * more than 90 degrees, though it may take multiple cycles to reach the desired
 * positions.
 */
public class SetModulePositionsRequest implements SwerveRequest {
    private Rotation2d[] positions;

    /**
     * Modifies the modulePositions parameter and returns itself.
     *
     * @param modulePositions - Parameter to modify.
     *
     * @return this object
     */
    public SetModulePositionsRequest withModulePositions(final Rotation2d... modulePositions) {
        this.positions = modulePositions;

        return this;
    }

    @Override
    public StatusCode apply(
            SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        IntStream.range(0, modulesToApply.length).forEach(index -> {
            final SwerveModule<?, ?, ?> module = modulesToApply[index];
            final Rotation2d targetPosition =
                    calculateTargetModulePosition(module.getCurrentState().angle, positions[index]);
            final SwerveModule.ModuleRequest request =
                    new SwerveModule.ModuleRequest().withState(new SwerveModuleState(0, targetPosition));

            module.apply(request);
        });
        return StatusCode.OK;
    }

    private static Rotation2d calculateTargetModulePosition(
            Rotation2d currentPosition, final Rotation2d targetPosition) {
        final double currentAngle = currentPosition.getDegrees();
        final double targetAngle = targetPosition.getDegrees();
        final double error = targetAngle - currentAngle;

        if (Math.abs(error) <= 90) {
            return targetPosition;
        }

        final double newTargetAngle = targetAngle - Math.copySign(90, error);

        return Rotation2d.fromDegrees(newTargetAngle);
    }
}
