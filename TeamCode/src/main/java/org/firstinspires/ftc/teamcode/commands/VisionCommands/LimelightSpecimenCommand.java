package org.firstinspires.ftc.teamcode.commands.VisionCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

public class LimelightSpecimenCommand extends CommandBase {
    private LimelightSubsystem limelightSubsystem;
    private LLResult lastResult;
    private Follower follower;

    public LimelightSpecimenCommand(LimelightSubsystem limelightSubsystem, Follower follower){
        this.limelightSubsystem = limelightSubsystem;
        this.follower = follower;

        limelightSubsystem.setPipeline(5);
        addRequirements(limelightSubsystem);
    }
    @Override
    public void execute(){
        lastResult = limelightSubsystem.lookForSpecimen();
        limelightSubsystem.getLimelightTelemetrySpecimen(follower);



    }

}

