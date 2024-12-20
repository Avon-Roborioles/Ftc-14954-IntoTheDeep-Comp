package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class CancelCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    private PassSubsystem pass;
    private LiftSubsystem liftSubsystem;
    public CancelCommand(IntakeSubsystem subsystem, PassSubsystem pass, LiftSubsystem liftSubsystem) {
        this.subsystem = subsystem;
        this.pass = pass;
        this.liftSubsystem = liftSubsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.stopMotor();
        pass.PassOff();
        liftSubsystem.stopLift();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}