package org.firstinspires.ftc.teamcode.commands.LeverCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;

public class LeverClearCommand extends CommandBase {
    private LeverSubsystem lever;
    public LeverClearCommand(LeverSubsystem lever){
        this.lever = lever;
        addRequirements(lever);
    }
    @Override
    public void initialize(){
        lever.leverClear();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
