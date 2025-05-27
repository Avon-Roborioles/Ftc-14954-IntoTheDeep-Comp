package org.firstinspires.ftc.teamcode.commands.LightCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;

public class LightRedAlliance extends CommandBase {
    private NewIntakeSubsystem intake;
    public LightRedAlliance(NewIntakeSubsystem intake) {
        this.intake = intake;
    }
    public void initialize() {
        intake.redAllianceLight();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
