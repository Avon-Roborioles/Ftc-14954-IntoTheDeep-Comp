package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class NewStopCommand extends CommandBase {
    private NewIntakeSubsystem subsystem;

    public NewStopCommand(NewIntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        subsystem.stopMotor();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
