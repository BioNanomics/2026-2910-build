package org.frc2910.robot.subsystems.base.roller;

import org.frc2910.robot.RobotState;
import org.frc2910.robot.subsystems.base.MotorIO;
import org.frc2910.robot.subsystems.base.MotorIOInputsAutoLogged;
import org.frc2910.robot.subsystems.base.MotorIOTalonFX;
import org.frc2910.robot.util.ConditionalCallbackManager;
import org.frc2910.robot.util.DelayedBatchSignalLogger;
import org.littletonrobotics.junction.Logger;

import java.util.List;

/**
 * Extension of RollerMotorSubsystem that handles the case where there are multiple motors working together to power one joint in a mechanism. It functions with a leader motor and follower motor(s)
 * @see RollerMotorSubsystem
 * @see RollerWithFollowersSubsystemConfig
 * @apiNote YOU MUST use RollerWithFollowersSubsystemConfig when creating the leaderIO
 */
public class RollerWithFollowersSubsystem extends RollerMotorSubsystem {
    private final RollerWithFollowersSubsystemConfig leaderConfig;
    private final List<RollerWithFollowersSubsystemConfig.FollowerConfig> followerConfigs;
    private final MotorIOInputsAutoLogged[] followerInputs;
    private final MotorIO[] followerIOs;
    private final int amountOfFollowers;
    private final String logkeyFollower;

    public RollerWithFollowersSubsystem(
            RobotState robotState, ConditionalCallbackManager callbackManager, MotorIO leaderIO) {
        super(robotState, callbackManager, leaderIO);
        DelayedBatchSignalLogger.getInstance()
                .writeString(
                        "Phoenix6/TalonFX-"
                                + leaderIO.getConfig().getTalonCANID().getDeviceNumber() + "-Name",
                        leaderIO.getConfig().getMotorName());
        this.leaderConfig = (RollerWithFollowersSubsystemConfig) leaderIO.getConfig();
        this.followerConfigs = leaderConfig.getFollowerConfigs();
        this.amountOfFollowers = leaderConfig.getFollowerConfigs().size();
        this.followerInputs = new MotorIOInputsAutoLogged[this.amountOfFollowers];
        this.followerIOs = new MotorIOTalonFX[this.amountOfFollowers];
        for (int i = 0; i < amountOfFollowers; i++) {
            followerIOs[i] = new MotorIOTalonFX(followerConfigs.get(i), callbackManager);
            DelayedBatchSignalLogger.getInstance()
                    .writeString(
                            "Phoenix6/TalonFX-"
                                    + followerIOs[i].getConfig().getTalonCANID().getDeviceNumber() + "-Name",
                            followerIOs[i].getConfig().getMotorName());
            followerInputs[i] = new MotorIOInputsAutoLogged();
            MotorIO followerIO = followerIOs[i];
            followerIO.follow(
                    leaderConfig.getTalonCANID(), followerConfigs.get(i).getMotorAlignment());
        }
        logkeyFollower = motorName + "/follower";
        callbackManager.registerCallback(() -> {
            for (int i = 0; i < amountOfFollowers; i++) {
                Logger.processInputs(logkeyFollower + i, followerInputs[i]);
            }
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        for (int i = 0; i < amountOfFollowers; i++) {
            followerIOs[i].updateInputs(followerInputs[i]);
        }
    }

    public RollerWithFollowersSubsystemConfig getLeaderConfig() {
        return leaderConfig;
    }

    public List<RollerWithFollowersSubsystemConfig.FollowerConfig> getFollowerConfigs() {
        return followerConfigs;
    }

    public MotorIOInputsAutoLogged[] getFollowerInputs() {
        return followerInputs;
    }

    public MotorIO[] getFollowerIOs() {
        return followerIOs;
    }

    public int getAmountOfFollowers() {
        return amountOfFollowers;
    }
}
