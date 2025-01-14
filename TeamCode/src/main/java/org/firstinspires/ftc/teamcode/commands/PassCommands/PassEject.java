package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class PassEject extends CommandBase {
    private PassSubsystem pass;
    Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);

    public PassEject(PassSubsystem pass) {
        this.pass = pass;
        addRequirements(pass);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        pass.PassOn();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        pass.PassOff();
    }

}
