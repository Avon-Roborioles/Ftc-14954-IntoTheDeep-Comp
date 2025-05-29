package org.firstinspires.ftc.teamcode.commands.ExtendCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class ExtensionCommand extends CommandBase {
    private ExtendSubsystem extensionSubsystem;
    private double targetPosition;

    public ExtensionCommand(ExtendSubsystem extensionSubsystem, double targetPosition) {
        this.extensionSubsystem = extensionSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(extensionSubsystem);
    }

    @Override
    public void initialize() {
        extensionSubsystem.setPosition(targetPosition);
    }
    public boolean isFinished() {
        return !extensionSubsystem.isBusy();
    }

}
