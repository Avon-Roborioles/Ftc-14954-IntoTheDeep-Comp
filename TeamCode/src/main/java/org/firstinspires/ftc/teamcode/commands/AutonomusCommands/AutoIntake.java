package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class AutoIntake extends SequentialCommandGroup {


    public AutoIntake(NewIntakeSubsystem intake, WristSubsystem wrist){
        addCommands(
                new LowerWrist(wrist),
                new AutoCollectNoColorSample(intake, wrist)
        );
    }
}
