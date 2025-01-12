package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoIntake extends SequentialCommandGroup {


    public AutoIntake(IntakeSubsystem intake, WristSubsystem wrist){
        addCommands(
                new LowerWrist(wrist),
                new CollectSample(intake, wrist)

        );
    }
}
