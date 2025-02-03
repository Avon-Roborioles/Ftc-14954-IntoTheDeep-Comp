package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

import java.util.concurrent.TimeUnit;

public class PassOnToEndCommand extends SequentialCommandGroup {
    private PassSubsystem pass;
    private IntakeSubsystem intake;
    Timing.Timer timer = new Timing.Timer(3, TimeUnit.SECONDS);
    public PassOnToEndCommand(PassSubsystem pass, IntakeSubsystem intake){
        this.pass = pass;
        this.intake = intake;
        addRequirements(pass);
    }

    @Override
    public void initialize(){pass.PassMotorControl(0.5);
        timer.start();
        if(pass.IsPassDistanceSensorCooked()){
        intake.PassFailLight();
        }
    }
    @Override
    public void end(boolean interrupted){pass.PassOff();
    }
    @Override
    public boolean isFinished(){
        if(pass.IsPassDistanceSensorCooked()){
            return timer.done();
        }else{
            return pass.PassDistanceTrue();
        }

    }


}
