package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

import java.util.concurrent.TimeUnit;

public class PassBack extends CommandBase {
    private PassSubsystem passSubsystem;
    Timing.Timer timer = new Timing.Timer(3, TimeUnit.SECONDS);
    public PassBack(PassSubsystem passSubsystem){
        this.passSubsystem = passSubsystem;
        addRequirements(passSubsystem);
    }
    @Override
    public void initialize(){
        timer.start();
    }
    @Override
    public void execute(){
        passSubsystem.passBack();
    }

    @Override
    public void end(boolean interrupted){
        passSubsystem.PassOff();
    }
    @Override
    public boolean isFinished(){
        return timer.done();
    }
}
