package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class PassOnToEndCommand extends SequentialCommandGroup {
    private PassSubsystem pass;
    private IntakeSubsystem intake;

    public PassOnToEndCommand(PassSubsystem pass, IntakeSubsystem intake){
        this.pass = pass;
        this.intake = intake;
        addRequirements(pass);
    }

    @Override
    public void initialize(){pass.PassMotorControl(0.5);}
    @Override
    public void end(boolean interrupted){pass.PassOff();
    }
    @Override
    public boolean isFinished(){return pass.PassDistanceTrue();}


}
