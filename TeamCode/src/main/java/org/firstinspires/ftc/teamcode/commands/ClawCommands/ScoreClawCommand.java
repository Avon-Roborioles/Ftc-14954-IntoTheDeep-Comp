package org.firstinspires.ftc.teamcode.commands.ClawCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import java.util.concurrent.TimeUnit;

public class ScoreClawCommand extends CommandBase {
    private ClawSubsystem claw;
    private Timing.Timer timer = new Timing.Timer(250, TimeUnit.MILLISECONDS);

    public ScoreClawCommand(ClawSubsystem claw){
        this.claw = claw;
        addRequirements(claw);
    }
    @Override
    public void initialize(){
        claw.open();
        timer.start();
    }
    @Override
    public boolean isFinished(){
        return timer.done();
    }
    @Override
    public void end(boolean interrupted){
    }
}
