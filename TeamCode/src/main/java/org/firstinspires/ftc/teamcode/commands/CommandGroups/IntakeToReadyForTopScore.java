package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassToBox;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.WristClearBar;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IntakeToReadyForTopScore extends SequentialCommandGroup {
    private IntakeSubsystem intake;
    private WristSubsystem wrist;
    private PassSubsystem pass;
    private LiftSubsystem lift;
    private ExtendSubsystem extend;
    private SwingArmSubsystem swingArm;
    private BoxxySubsystem box;

    public IntakeToReadyForTopScore(IntakeSubsystem intake, WristSubsystem wrist, PassSubsystem pass, ExtendSubsystem extend, SwingArmSubsystem swingArm, BoxxySubsystem box, LiftSubsystem lift){
        addCommands(
                new LowerWrist(wrist),
                new CollectSample(intake, wrist),
                new WristClearBar(wrist),
                new RetractCommand(extend),
                new HandoffCommand(wrist),
                new PassToBox(pass, box, intake),
                new TopBucketScoreReady(swingArm, lift, pass)
        );
    }
}
