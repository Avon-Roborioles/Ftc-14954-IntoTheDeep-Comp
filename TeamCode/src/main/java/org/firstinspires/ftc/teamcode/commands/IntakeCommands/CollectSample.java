package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class CollectSample extends CommandBase {
    private NewIntakeSubsystem subsystem;
    private boolean first=true;
    private boolean stalled = false;
    Timing.Timer stall = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    public CollectSample(NewIntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }


    @Override
    public void execute() {
        if (!stalled){
            if (!subsystem.isStalled()){
                if (subsystem.hasSample()){
                    subsystem.stopMotor();
                    if(!subsystem.correctSample()){
                        subsystem.runMotor();
                    }
                }else {
                    subsystem.runMotor();
                    first = true;
                }

            }else {
                if (first){
                    stall.start();
                    subsystem.stopMotor();
                    first=false;
                    stalled = true;
                }

                subsystem.rejectMotor();

            }
        }else {
            if (stall.done()){
                subsystem.stopMotor();
                stalled = false;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();

    }

    @Override
    public boolean isFinished() {
        return subsystem.correctSample();
    }
}

