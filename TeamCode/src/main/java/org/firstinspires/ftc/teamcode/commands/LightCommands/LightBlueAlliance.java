package org.firstinspires.ftc.teamcode.commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class LightBlueAlliance extends CommandBase {
    private IntakeSubsystem intake;
    public LightBlueAlliance(IntakeSubsystem intake) {
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
