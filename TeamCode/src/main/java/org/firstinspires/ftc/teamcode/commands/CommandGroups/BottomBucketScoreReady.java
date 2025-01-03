package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomBucketCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class BottomBucketScoreReady extends SequentialCommandGroup {
    public BottomBucketScoreReady(SwingArmSubsystem swingArm, LiftSubsystem lift){
        addCommands(
                new LiftBottomBucketCommand(lift),
                new SwingArmMidCommand(swingArm)
        );
    }
}
