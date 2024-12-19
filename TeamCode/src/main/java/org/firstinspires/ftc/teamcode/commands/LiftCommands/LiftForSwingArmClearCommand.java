package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftForSwingArmClearCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    public LiftForSwingArmClearCommand(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
    @Override
    public void execute () {
        liftSubsystem.setSwingarmClearPosition();
    }
    @Override
    public boolean isFinished () {
        return !liftSubsystem.isBusy();
    }
    @Override
    public void end(boolean interrupted){
        liftSubsystem.stopLift();
    }
}
