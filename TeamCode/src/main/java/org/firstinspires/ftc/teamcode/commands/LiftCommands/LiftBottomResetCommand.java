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
        liftSubsystem.setPower(-0.5);
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
