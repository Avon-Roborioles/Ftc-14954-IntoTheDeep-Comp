package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class PassToEnd extends ParallelCommandGroup {
    public PassToEnd(PassSubsystem pass, IntakeSubsystem intake){
        addCommands(
                new EjectCommand(intake, pass),
                new PassOnToEndCommand(pass, intake)
        );
    }
}
