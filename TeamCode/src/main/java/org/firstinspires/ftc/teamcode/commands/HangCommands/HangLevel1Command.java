package org.firstinspires.ftc.teamcode.commands.HangCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class HangLevel1Command extends SequentialCommandGroup {
    private LiftSubsystem lift;
    private SwingArmSubsystem arm;
    public HangLevel1Command(LiftSubsystem lift, SwingArmSubsystem arm){
        addCommands(
                new LiftForSwingArmClearCommand(lift),
                new SwingArmUpCommand(arm),
                new WaitCommand(250),
                new LiftBottomCommand(lift)
        );
    }
}
