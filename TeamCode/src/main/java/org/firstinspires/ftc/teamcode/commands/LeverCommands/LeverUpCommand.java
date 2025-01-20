package org.firstinspires.ftc.teamcode.commands.LeverCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;

public class LeverUpCommand extends CommandBase {
    private LeverSubsystem lever;

    public LeverUpCommand(LeverSubsystem lever){
        this.lever = lever;
        addRequirements(lever);

    }

    @Override
    public void initialize(){
        lever.leverUp();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
