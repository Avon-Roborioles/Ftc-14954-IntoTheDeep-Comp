package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendClearGearCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SpitCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoIntake extends SequentialCommandGroup {


    public AutoIntake(NewIntakeSubsystem intake, WristSubsystem wrist, ExtendSubsystem extend){
        addCommands(
                new ExtendClearGearCommand(extend),
                new LowerWrist(wrist),
                new AutoCollectNoColorSample(intake, wrist),
                new SpitCommand(intake, wrist)
        );
    }
    public AutoIntake(NewIntakeSubsystem intake, WristSubsystem wrist){
        addCommands(
                new LowerWrist(wrist),
                new AutoCollectNoColorSample(intake, wrist)//,
//                new SpitCommand(intake, wrist)
        );
    }
}
