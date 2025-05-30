package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoCollectNoColorSample extends CommandBase {
    private NewIntakeSubsystem subsystem;
    private WristSubsystem wrist;
    private boolean hasSample = false;
    private boolean validSample = false;
    boolean ejecting = false;
    Timing.Timer timer = new Timing.Timer(4, TimeUnit.SECONDS);

    // This command will acquire samples and either eject them or accept them based on color
    public AutoCollectNoColorSample(NewIntakeSubsystem subsystem, WristSubsystem wrist) {
        this.subsystem = subsystem;
        this.wrist = wrist;
        subsystem.setSkipLastSample(false);
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        hasSample = false;
        validSample = false;
        ejecting = false;
        subsystem.setSkipLastSample(false);
        timer.start();
    }

    @Override
    public void execute() {
        hasSample = subsystem.hasSample();
        if (!timer.done()) {
            if (!hasSample) {
                subsystem.runMotor();
            } else {
                validSample = true;
                if (subsystem.getRedAlliance() & subsystem.isColorSensorRed()) {
                    subsystem.redlight();
                } else if (!subsystem.getRedAlliance() & subsystem.isColorSensorBlue()) {
                    subsystem.bluelight();
                } else {
                    subsystem.yellowlight();
                }
            }
        }else{
            validSample = true;
            subsystem.setSkipLastSample(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
        // Exit if command is interrupted
        if(interrupted) validSample = true;
    }

    @Override
    public boolean isFinished() {
//        return true;
        return validSample;
    }
}

