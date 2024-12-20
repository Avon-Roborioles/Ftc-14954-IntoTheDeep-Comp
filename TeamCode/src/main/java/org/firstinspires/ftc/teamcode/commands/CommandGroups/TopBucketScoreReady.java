package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassOffCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class TopBucketScoreReady extends SequentialCommandGroup {
    public TopBucketScoreReady(SwingArmSubsystem swingArm, LiftSubsystem lift, PassSubsystem pass){
        addCommands(
                new PassOffCommand(pass),
                new LiftForSwingArmClearCommand(lift),
                new SwingArmMidCommand(swingArm),
                new LiftTopCommand(lift)
        );
    }
}
