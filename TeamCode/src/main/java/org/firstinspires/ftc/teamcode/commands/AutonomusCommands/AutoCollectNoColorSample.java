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
    private boolean first=true;
    private boolean validSample = false;
    boolean ejecting = false;
    private boolean stalled = false;
    Timing.Timer stall = new Timing.Timer(1000, TimeUnit.MILLISECONDS);

    Timing.Timer timer;

    // This command will acquire samples and either eject them or accept them based on color
    public AutoCollectNoColorSample(NewIntakeSubsystem subsystem, WristSubsystem wrist, long timeout) {
        this.subsystem = subsystem;
        this.wrist = wrist;
        timer = new Timing.Timer(timeout, TimeUnit.MILLISECONDS);
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

        if (!stalled){
            if (!subsystem.isStalled()){
                first = true;

                hasSample = subsystem.hasSample();
                if (!timer.done()) {
                    if (!hasSample) {
                        subsystem.runMotor();
                    } else {
                        validSample = true;
                        subsystem.setSkipLastSample(false);
                    }
                }else{
                    validSample = true;
                    subsystem.setSkipLastSample(true);
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
        // Exit if command is interrupted
        if(interrupted) validSample = true;
    }

    @Override
    public boolean isFinished() {
//        return true;
        return validSample;
    }
}

