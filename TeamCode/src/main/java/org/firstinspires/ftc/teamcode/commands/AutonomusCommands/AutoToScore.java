package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.CommandGroups.TopBucketScoreReady;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ColorResetCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassToBox;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.WristClearBar;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoToScore extends SequentialCommandGroup {

    public AutoToScore(IntakeSubsystem intake, WristSubsystem wrist, PassSubsystem pass, ExtendSubsystem extend, SwingArmSubsystem swingArm, BoxxySubsystem box, LiftSubsystem lift){
        addCommands(
                new ParallelCommandGroup(
                        new RetractCommand(extend),
                        new HandoffCommand(wrist)
                ),
                new LiftBottomCommand(lift),
                new PassToBox(pass, box, intake),
                new TopBucketScoreReady(swingArm, lift, pass),
                new SwingArmScoreCommand(swingArm, box, intake),
                new ColorResetCommand(intake)
        );
    }
}
