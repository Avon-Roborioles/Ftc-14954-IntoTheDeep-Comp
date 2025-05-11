package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class CollectSample extends CommandBase {
    private IntakeSubsystem subsystem;
    private WristSubsystem wrist;
    private boolean hasSample = false;
    private boolean validSample = false;
    boolean ejecting = false;
    Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);

    // This command will acquire samples and either eject them or accept them based on color
    public CollectSample(IntakeSubsystem subsystem, WristSubsystem wrist) {
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

        if (!hasSample & !ejecting) {
            subsystem.runMotor();
        }
        if (ejecting & !timer.done()) {
            subsystem.runMotor();
        } else if (ejecting & timer.done()) {
            ejecting = false;
            subsystem.stopMotor();
            subsystem.rainbowlight();
            wrist.down();
        }

        if (hasSample & !ejecting) {
            // check if sample is valid
            if ((subsystem.getRedAlliance() & subsystem.isColorSensorBlue()))  {
                validSample = false;
                subsystem.rejectMotor();
                wrist.middle();
                if (subsystem.isColorSensorBlue()){
                    subsystem.bluelight();
                } else {
                    subsystem.yellowlight();
                }
                ejecting = true;
                timer.start();
            } else if ((!subsystem.getRedAlliance() & subsystem.isColorSensorRed())) {
                validSample = false;
                wrist.middle();
                subsystem.rejectMotor();


                if (subsystem.isColorSensorRed()){
                    subsystem.redlight();
                } else {
                    subsystem.yellowlight();
                }
                ejecting = true;
                timer.start();
            } else {
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

