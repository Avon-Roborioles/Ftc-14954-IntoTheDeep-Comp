package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class PassOffCommand extends CommandBase {
    private PassSubsystem pass;
    public PassOffCommand(PassSubsystem pass){
        this.pass = pass;
        addRequirements(pass);
    }
    @Override
    public void execute(){
        pass.PassOff();
    }
    @Override
    public boolean isFinished(){return true;}

}
