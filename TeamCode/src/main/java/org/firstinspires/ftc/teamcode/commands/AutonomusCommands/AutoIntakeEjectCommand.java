package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoIntakeEjectCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    private PassSubsystem pass;
    Timing.Timer timer = new Timing.Timer(4, TimeUnit.SECONDS);
    public AutoIntakeEjectCommand(IntakeSubsystem subsystem, PassSubsystem pass) {
        this.subsystem = subsystem;
        this.pass = pass;
        addRequirements(subsystem);
    }



    @Override
    public void execute() {
        subsystem.runMotor();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        if(subsystem.getSkipLastSample()){
            return true;
        }else {
            if (pass.IsPassDistanceSensorCooked()) {
                return timer.done();
            } else {
                return pass.PassDistanceTrue();
            }
        }
    }
}

