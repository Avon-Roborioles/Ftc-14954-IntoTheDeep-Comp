package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoAfterScore extends SequentialCommandGroup {
    private SwingArmSubsystem swingArm;
    private LiftSubsystem lift;
    public AutoAfterScore(SwingArmSubsystem swingArm, LiftSubsystem lift){
        addCommands(
                new SwingArmDownCommand(swingArm),
                new LiftBottomCommand(lift)
        );
    }
}
