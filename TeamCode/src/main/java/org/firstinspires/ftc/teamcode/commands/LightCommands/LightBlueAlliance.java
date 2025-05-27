package org.firstinspires.ftc.teamcode.commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;

public class LightBlueAlliance extends CommandBase {
    private NewIntakeSubsystem intake;
    public LightBlueAlliance(NewIntakeSubsystem intake) {
        this.intake = intake;
    }
    public void initialize() {
        intake.blueAllianceLight();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
