package org.firstinspires.ftc.teamcode.commands.ExtendCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class ExtendClearGearCommand extends CommandBase {
    private ExtendSubsystem extendSubsystem;

    public ExtendClearGearCommand(ExtendSubsystem extendSubsystem){
        this.extendSubsystem = extendSubsystem;
        addRequirements(extendSubsystem);
    }

    @Override
    public void initialize () {
        if (extendSubsystem.retracted()) {
            extendSubsystem.clear();
        }
    }

    @Override
    public boolean isFinished() { return true; }
}
