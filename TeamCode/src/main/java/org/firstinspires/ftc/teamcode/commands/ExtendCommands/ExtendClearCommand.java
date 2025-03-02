package org.firstinspires.ftc.teamcode.commands.ExtendCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class ExtendClearCommand extends CommandBase {
    private ExtendSubsystem extendSubsystem;

    public ExtendClearCommand (ExtendSubsystem extendSubsystem){
        this.extendSubsystem = extendSubsystem;
        addRequirements(extendSubsystem);
    }

    @Override
    public void initialize () {
        extendSubsystem.clear();
    }

    @Override
    public boolean isFinished() { return true; }
}
