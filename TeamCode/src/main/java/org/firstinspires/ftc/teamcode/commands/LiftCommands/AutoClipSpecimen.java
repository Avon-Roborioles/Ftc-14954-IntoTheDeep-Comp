package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class AutoClipSpecimen extends SequentialCommandGroup {
    public AutoClipSpecimen(LiftSubsystem lift, int wait){
        addCommands(
                new LiftTopHookCommand(lift),
                new WaitCommand(wait),
                new LiftForSwingArmClearCommand(lift)
        );
        addRequirements(lift);
    }
}
