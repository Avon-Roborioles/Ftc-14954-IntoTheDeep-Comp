package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;

public class ColorResetCommand extends CommandBase {
    private NewIntakeSubsystem subsystem;

    public ColorResetCommand(NewIntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.rainbowlight();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
