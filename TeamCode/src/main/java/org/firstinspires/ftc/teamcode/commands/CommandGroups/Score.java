package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ColorResetCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class Score extends SequentialCommandGroup {
    private SwingArmSubsystem swingArm;
    private LiftSubsystem lift;
    private BoxxySubsystem box;
    private IntakeSubsystem intake;

    public Score(SwingArmSubsystem swingArm, LiftSubsystem lift, BoxxySubsystem box, IntakeSubsystem intake){
        addCommands(
                new SwingArmScoreCommand(swingArm, box, intake),
                new ColorResetCommand(intake),
                new SwingArmDownCommand(swingArm),
                new LiftForSwingArmClearCommand(lift),
                new LiftBottomCommand(lift)
        );
    }
}
