package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftBottomResetCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    public LiftBottomResetCommand(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
    @Override
    public void execute () {
        liftSubsystem.setBottomPosition();
    }
    @Override
    public boolean isFinished () {
        return liftSubsystem.isBottom();
    }
    @Override
    public void end(boolean interrupted){
        liftSubsystem.resetEncoder();
        liftSubsystem.stopLift();
    }
}
