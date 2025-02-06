package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassOnToEndCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class AutoPassToEnd extends ParallelCommandGroup {
    public AutoPassToEnd(PassSubsystem pass, IntakeSubsystem intake){
        addCommands(
                new AutoIntakeEjectCommand(intake, pass),
                new AutoPassOnToEndCommand(pass, intake)
        );
    }
}
