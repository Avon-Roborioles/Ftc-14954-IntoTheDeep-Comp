package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftClearRampCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.AutoSwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class PreloadToScore extends SequentialCommandGroup {

    public PreloadToScore(SwingArmSubsystem swingArm, LiftSubsystem lift, NewIntakeSubsystem intake, ClawSubsystem claw){
        addCommands(
                new CloseClawCommand(claw),
                new WaitCommand(100),
                new AutoTopBucketScoreReady(swingArm, lift, intake)
        );
    }
}
