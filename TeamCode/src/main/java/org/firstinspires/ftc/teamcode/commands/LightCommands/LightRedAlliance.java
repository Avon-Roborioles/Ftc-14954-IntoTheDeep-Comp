package org.firstinspires.ftc.teamcode.commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class LightRedAlliance extends CommandBase {
    private IntakeSubsystem intake;
    public LightRedAlliance(IntakeSubsystem intake) {
        this.intake = intake;
    }
    public void initialize() {
        intake.redAllianceLight();
        intake.changeAlliance();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
