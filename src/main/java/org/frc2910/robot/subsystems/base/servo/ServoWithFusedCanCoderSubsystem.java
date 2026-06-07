package org.frc2910.robot.subsystems.base.servo;

import org.frc2910.robot.RobotState;
import org.frc2910.robot.subsystems.base.FusedCanCoderIO;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.util.ConditionalCallbackManager;

/**
 * Extension of ServoMotorSubsystem for the case of a servo subsystem that uses a CanCoder that is outside the talon's encoder. Useful for absolute position and tuning feedback
 * @apiNote NOT FINISHED, and YOU MUST use ServoWithFusedCanCoderSubsystemConfig when creating the IO
 * @see ServoMotorSubsystem
 * @see ServoWithFusedCanCoderSubsystemConfig
 */
public class ServoWithFusedCanCoderSubsystem extends ServoMotorSubsystem {
    private final FusedCanCoderIO fusedCanCoderIO;

    public ServoWithFusedCanCoderSubsystem(
            RobotState robotState,
            ConditionalCallbackManager callbackManager,
            ServoWithFusedCanCoderSubsystemConfig motorConfig,
            MotorIO motorIO,
            FusedCanCoderIO cIO) {
        super(robotState, callbackManager, motorIO);

        this.fusedCanCoderIO = cIO;
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public FusedCanCoderIO getFusedCanCoderIO() {
        return fusedCanCoderIO;
    }
}
