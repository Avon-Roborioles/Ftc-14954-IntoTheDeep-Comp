package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class PassOnToBoxCommand extends SequentialCommandGroup {
    private PassSubsystem pass;
    private BoxxySubsystem box;
    private IntakeSubsystem intake;

    public PassOnToBoxCommand(PassSubsystem pass, BoxxySubsystem box, IntakeSubsystem intake){
        this.pass = pass;
        this.box = box;
        this.intake = intake;
        addRequirements(pass, this.box);
    }

    @Override
    public void initialize(){pass.PassOn();}

    public void end(){pass.PassOff();
    }
    @Override
    public boolean isFinished(){return box.haveSample();}


}
