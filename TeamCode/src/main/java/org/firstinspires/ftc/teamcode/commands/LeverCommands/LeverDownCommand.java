package org.firstinspires.ftc.teamcode.commands.LeverCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;

public class LeverDownCommand extends CommandBase {
    private LeverSubsystem lever;

    public LeverDownCommand(LeverSubsystem lever){
        this.lever = lever;
        addRequirements(lever);

    }

    @Override
    public void initialize(){
        lever.leverDown();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
