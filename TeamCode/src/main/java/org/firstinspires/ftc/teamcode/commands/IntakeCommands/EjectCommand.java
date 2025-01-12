package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class EjectCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    private BoxxySubsystem box;

    public EjectCommand(IntakeSubsystem subsystem, BoxxySubsystem box) {
        this.subsystem = subsystem;
        this.box = box;
        addRequirements(subsystem);
    }



    @Override
    public void execute() {
        subsystem.runMotor();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();
        subsystem.rainbowlight();
    }

    @Override
    public boolean isFinished() {
        return box.haveSample();
    }
}

