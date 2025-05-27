package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class Reject extends CommandBase {
    private NewIntakeSubsystem intake;
    Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);
    public Reject(NewIntakeSubsystem intake){
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
