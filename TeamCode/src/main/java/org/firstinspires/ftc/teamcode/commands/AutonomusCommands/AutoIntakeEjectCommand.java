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
    Timing.Timer eject = new Timing.Timer(1, TimeUnit.SECONDS);
    public AutoIntakeEjectCommand(IntakeSubsystem subsystem, PassSubsystem pass) {
        this.subsystem = subsystem;
        this.pass = pass;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        if (subsystem.getSkipLastSample()){
            eject.start();
            subsystem.rejectMotor();
        }else {
            subsystem.runMotor();
            timer.start();
        }
    }



    @Override
    public void execute() {
        if (subsystem.getSkipLastSample()){
            subsystem.rejectMotor();
        }else {
            subsystem.runMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        if(subsystem.getSkipLastSample()){
            return eject.done();
        }else {
            if (pass.IsPassDistanceSensorCooked()) {
                return timer.done();
            } else {
                return pass.PassDistanceTrue();
            }
        }
    }
}

