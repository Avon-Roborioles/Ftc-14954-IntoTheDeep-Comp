package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendClearGearCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NomNomComand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.Reject;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftClearRampCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IntakeToReadyForTopScore extends SequentialCommandGroup {
    public IntakeToReadyForTopScore(NewIntakeSubsystem intake, WristSubsystem wrist, ExtendSubsystem extend, SwingArmSubsystem swingArm, LiftSubsystem lift, ClawSubsystem claw){
        addCommands(
                new ExtendClearGearCommand(extend),
                new LiftClearRampCommand(lift),
                new OpenClawCommand(claw),
                new NomNomComand(intake, wrist, extend),
                new ParallelCommandGroup(
                        new Reject(intake),
                        new SequentialCommandGroup(
                                new LiftBottomCommand(lift),
                                new WaitCommand(100),
                                new CloseClawCommand(claw),
                                new WaitCommand(100),
                                new LiftForSwingArmClearCommand(lift),
                                new SwingArmUpCommand(swingArm),
                                new LiftTopCommand(lift)
                        )
                )


        );
    }
}
