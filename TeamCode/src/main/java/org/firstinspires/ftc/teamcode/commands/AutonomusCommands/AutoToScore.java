package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ColorResetCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SpitCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftClearRampCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.AutoSwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoToScore extends SequentialCommandGroup {

    public AutoToScore(NewIntakeSubsystem intake, WristSubsystem wrist, ExtendSubsystem extend, SwingArmSubsystem swingArm, LiftSubsystem lift, ClawSubsystem claw){
        addCommands(
                new LiftClearRampCommand(lift),
                new ParallelCommandGroup(
                        new RetractCommand(extend),
                        new RaiseWrist(wrist),
                        new LiftClearRampCommand(lift),
                        new OpenClawCommand(claw),
                        new SpitCommand(intake, wrist)
                ),
                new LiftBottomCommand(lift),
                new CloseClawCommand(claw),
                new WaitCommand(100),
                new AutoTopBucketScoreReady(swingArm, lift, intake)
        );
    }


}
