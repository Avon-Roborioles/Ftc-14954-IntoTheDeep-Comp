package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class NomNomComand extends SequentialCommandGroup {

    public NomNomComand(NewIntakeSubsystem intake, WristSubsystem wrist, ExtendSubsystem extend) {
        addCommands(
                new LowerWrist(wrist),
                new CollectSample(intake),
                new SpitCommand(intake, wrist),
                new RetractCommand(extend)
        );
    }
}
