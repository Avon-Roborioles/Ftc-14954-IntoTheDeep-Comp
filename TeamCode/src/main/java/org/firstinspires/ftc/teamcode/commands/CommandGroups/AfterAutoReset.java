package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AfterAutoReset extends SequentialCommandGroup {
    public AfterAutoReset(LiftSubsystem liftSubsystem, SwingArmSubsystem swingArmSubsystem) {
        addCommands(
                new LiftForSwingArmClearCommand(liftSubsystem),
                new SwingArmDownCommand(swingArmSubsystem),
                new LiftBottomCommand(liftSubsystem)
        );
    }
}
