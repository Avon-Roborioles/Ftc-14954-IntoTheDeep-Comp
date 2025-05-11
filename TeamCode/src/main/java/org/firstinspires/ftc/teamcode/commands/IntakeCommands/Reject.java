package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class Reject extends CommandBase {
    private IntakeSubsystem intake;
    private WristSubsystem wrist;
    Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);
    public Reject(IntakeSubsystem intake, WristSubsystem wrist){
        this.intake = intake;
        this.wrist = wrist;
        addRequirements(wrist);
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        timer.start();
        wrist.down();
        //TODO GET THE NEW INTAKE WRIST VALUES

    }
    @Override
    public void execute() {
        intake.runMotor();
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
