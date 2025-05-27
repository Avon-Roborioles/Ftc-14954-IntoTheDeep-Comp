package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;

public class CancelCommand extends CommandBase {
    private NewIntakeSubsystem subsystem;
    private LiftSubsystem liftSubsystem;
    public CancelCommand(NewIntakeSubsystem subsystem, LiftSubsystem liftSubsystem) {
        this.subsystem = subsystem;
        this.liftSubsystem = liftSubsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.stopMotor();
        liftSubsystem.stopLift();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}