package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftTopBarCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;

    public LiftTopBarCommand(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.setTopBarPosition();
    }

    @Override
    public boolean isFinished() {
        return !liftSubsystem.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLift();
    }
}
