package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class Reject extends CommandBase {
    private IntakeSubsystem intake;
    Timing.Timer timer = new Timing.Timer(2, TimeUnit.SECONDS);
    public Reject(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        timer.start();
    }
    @Override
    public void execute() {
        intake.rejectMotor();
    }
    @Override
    public boolean isFinished() {
        return timer.done();
    }
    @Override
    public void end(boolean interrupted) {
        intake.stopMotor();
    }


}
