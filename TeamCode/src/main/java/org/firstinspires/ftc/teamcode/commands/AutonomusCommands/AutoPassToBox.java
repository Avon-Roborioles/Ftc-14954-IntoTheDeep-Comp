package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassOnToBoxCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class AutoPassToBox extends ParallelCommandGroup {
    public AutoPassToBox(PassSubsystem pass, BoxxySubsystem box, IntakeSubsystem intake){
        addCommands(
                new AutoIntakeEjectCommand(intake, pass),
                new AutoPassOnToBoxCommand(pass,box, intake)
        );
    }
}
