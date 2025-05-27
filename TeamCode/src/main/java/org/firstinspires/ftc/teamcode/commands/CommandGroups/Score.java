package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.ScoreClawCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftClearRampCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class Score extends SequentialCommandGroup {
    public Score(SwingArmSubsystem swingArm, LiftSubsystem lift, ClawSubsystem claw, NewIntakeSubsystem intake){
        addCommands(
                new ScoreClawCommand(claw),
                new WaitCommand(250),
                new LiftForSwingArmClearCommand(lift),
                new SwingArmDownCommand(swingArm),
                new LiftClearRampCommand(lift)
        );
    }
}
