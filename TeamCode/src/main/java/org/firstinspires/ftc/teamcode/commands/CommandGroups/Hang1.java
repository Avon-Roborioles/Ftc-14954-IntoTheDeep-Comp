package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmParkCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class Hang1 extends SequentialCommandGroup {
    private LiftSubsystem lift;
    private SwingArmSubsystem arm;
    public Hang1 (LiftSubsystem lift, SwingArmSubsystem arm){
        addCommands(
                new LiftForSwingArmClearCommand(lift),
                new SwingArmUpCommand(arm),
                new WaitCommand(250),
                new LiftBottomCommand(lift)
        );
    }
}
