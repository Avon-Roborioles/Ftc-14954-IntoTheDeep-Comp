package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class ClipTopSpecimen extends SequentialCommandGroup {
    public ClipTopSpecimen (LiftSubsystem lift, int wait){
        addCommands(
                new LiftTopHookCommand(lift),
                new WaitCommand(wait),
                new LiftBottomCommand(lift)
        );
        addRequirements(lift);
    }
}
