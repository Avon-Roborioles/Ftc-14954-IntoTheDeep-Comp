package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoCollectNoColorSample;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class NomNomComand extends SequentialCommandGroup {

    public NomNomComand(NewIntakeSubsystem intake, WristSubsystem wrist, Telemetry telemetry) {
        addCommands(
                new LowerWrist(wrist),
                new NewCollectCommand(intake, wrist, telemetry),
                new NewSpitCommand(intake, wrist)
        );
    }
}
