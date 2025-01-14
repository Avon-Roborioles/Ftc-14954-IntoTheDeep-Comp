package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class EjectCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    private PassSubsystem pass;

    public EjectCommand(IntakeSubsystem subsystem, PassSubsystem pass) {
        this.subsystem = subsystem;
        this.pass = pass;
        addRequirements(subsystem);
    }



    @Override
    public void execute() {
        subsystem.runMotor();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return pass.PassDistanceTrue();
    }
}

