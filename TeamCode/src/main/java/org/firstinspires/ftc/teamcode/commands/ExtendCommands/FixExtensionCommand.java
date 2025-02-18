package org.firstinspires.ftc.teamcode.commands.ExtendCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class FixExtensionCommand extends CommandBase {
    private ExtendSubsystem subsystem;
    public FixExtensionCommand(ExtendSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.disable();
    }
    @Override
    public boolean isFinished() {
        return subsystem.retracted();
    }
    @Override
    public void end(boolean interrupted) {
        subsystem.enable();
    }
}
