package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;

public class ToggleAlliance extends CommandBase {
    private NewIntakeSubsystem subsystem;

    public ToggleAlliance(NewIntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.changeAlliance();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
