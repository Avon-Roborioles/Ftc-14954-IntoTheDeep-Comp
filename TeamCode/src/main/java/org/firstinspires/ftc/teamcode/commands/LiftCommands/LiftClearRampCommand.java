package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftClearRampCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    public LiftClearRampCommand(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
    @Override
    public void initialize() {
        liftSubsystem.setClearRampPosition();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLift();
    }
}
