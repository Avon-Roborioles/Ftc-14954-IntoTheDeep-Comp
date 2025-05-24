package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class NewSpitCommand extends CommandBase {
    private NewIntakeSubsystem subsystem;
    private WristSubsystem wrist;

    Timing.Timer timer = new Timing.Timer(450, TimeUnit.MILLISECONDS);

    public NewSpitCommand(NewIntakeSubsystem subsystem, WristSubsystem wrist) {
        this.subsystem = subsystem;
        this.wrist = wrist;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        wrist.up();
        if (timer.done()){
            subsystem.runMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
        if (subsystem.isRampSensorRed()){
            subsystem.redlight();
        } else if (subsystem.isRampSensorBlue()){
            subsystem.bluelight();
        } else if (subsystem.isRampSensorYellow()){
            subsystem.yellowlight();
        }




    }

    @Override
    public boolean isFinished() {
        return subsystem.inRamp();
    }

}
