package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class PreloadToScore extends SequentialCommandGroup {

    public PreloadToScore(SwingArmSubsystem swingArm, BoxxySubsystem box, LiftSubsystem lift, IntakeSubsystem intake){
        addCommands(
                new LiftForSwingArmClearCommand(lift),
                new SwingArmMidCommand(swingArm),
                new LiftTopCommand(lift),
                new SwingArmScoreCommand(swingArm, box, intake)
        );
    }
}
