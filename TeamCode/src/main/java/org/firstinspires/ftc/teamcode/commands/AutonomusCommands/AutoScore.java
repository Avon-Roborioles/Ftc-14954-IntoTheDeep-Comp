package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ColorResetCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.AutoSwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(NewIntakeSubsystem intake, SwingArmSubsystem swingArm, ClawSubsystem claw){
        addCommands(
                new AutoSwingArmScoreCommand(swingArm, intake, claw),
                new ColorResetCommand(intake)
        );

    }
}
