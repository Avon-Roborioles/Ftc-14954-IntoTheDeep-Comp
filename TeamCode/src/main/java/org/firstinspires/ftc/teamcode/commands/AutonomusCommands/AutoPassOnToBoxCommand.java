package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoPassOnToBoxCommand extends SequentialCommandGroup {
    private PassSubsystem pass;
    private BoxxySubsystem box;
    private IntakeSubsystem intake;
    Timing.Timer timer = new Timing.Timer(4, TimeUnit.SECONDS);
    public AutoPassOnToBoxCommand(PassSubsystem pass, BoxxySubsystem box, IntakeSubsystem intake){
        this.pass = pass;
        this.box = box;
        this.intake = intake;
        addRequirements(pass, this.box);
    }

    @Override
    public void initialize(){pass.PassOn();
        timer.start();
        if(box.IsBoxDistanceSensorCooked()){
            intake.BoxFailLight();
        }
    }

    public void end(){pass.PassOff();
    }
    @Override
    public boolean isFinished(){
        if(intake.getSkipLastSample()){
            return true;
        }else {
            if (box.IsBoxDistanceSensorCooked()) {
                return timer.done();
            } else {
                return box.haveSample();
            }
        }
    }


}
