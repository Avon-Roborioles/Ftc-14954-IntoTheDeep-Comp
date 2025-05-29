package org.firstinspires.ftc.teamcode.commands.ExtendCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

import java.util.concurrent.TimeUnit;

public class ExtendClearGearCommand extends CommandBase {
    private ExtendSubsystem extendSubsystem;
    boolean needsMove;
    Timing.Timer timer = new Timing.Timer(100, TimeUnit.MILLISECONDS);
    public ExtendClearGearCommand(ExtendSubsystem extendSubsystem){
        this.extendSubsystem = extendSubsystem;
        needsMove = false;
        addRequirements(extendSubsystem);
    }

    @Override
    public void initialize () {
        if (extendSubsystem.retracted()) {
            timer.start();
            needsMove = true;

            extendSubsystem.clear();
        }
    }

    @Override
    public boolean isFinished() {
        if (needsMove) {
            return timer.done();
        } else {
            return true;
        }
    }
}
