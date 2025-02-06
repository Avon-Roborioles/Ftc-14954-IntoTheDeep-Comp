package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoCollectNoColorSample extends CommandBase {
    private IntakeSubsystem subsystem;
    private WristSubsystem wrist;
    private boolean hasSample = false;
    private boolean validSample = false;
    boolean ejecting = false;
    Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);

    // This command will acquire samples and either eject them or accept them based on color
    public AutoCollectNoColorSample(IntakeSubsystem subsystem, WristSubsystem wrist) {
        this.subsystem = subsystem;
        this.wrist = wrist;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        hasSample = false;
        validSample = false;
        ejecting = false;
    }

    @Override
    public void execute() {
        hasSample = subsystem.hasSample();
        if (!hasSample) {
            subsystem.runMotor();
        }else{
            validSample = true;
            if (subsystem.getRedAlliance() & subsystem.isColorSensorRed()) {
                subsystem.redlight();
            } else if (!subsystem.getRedAlliance()& subsystem.isColorSensorBlue()) {
                subsystem.bluelight();
            } else {
                subsystem.yellowlight();
            }
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
        return validSample;
    }
}

