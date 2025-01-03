package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.oldPedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.oldPedroPathing.localization.Pose;

public class AutoSetStartCommand extends CommandBase {
    private Pose startPose;
    private Follower follower;

    public AutoSetStartCommand(Pose startPose, Follower follower) {
        this.startPose = startPose;
        this.follower = follower;
    }
    @Override
    public void execute() {
        follower.setPose(startPose);
    }
    @Override
    public boolean isFinished() {
//        if (follower.deviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
//            return true;
//        } else {
//            return false;
//        }
        return true;
    }
}
