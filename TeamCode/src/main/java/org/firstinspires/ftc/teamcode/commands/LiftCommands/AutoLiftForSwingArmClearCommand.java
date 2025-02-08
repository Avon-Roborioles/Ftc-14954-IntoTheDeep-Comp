package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class AutoLiftForSwingArmClearCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    private IntakeSubsystem intake;
    public AutoLiftForSwingArmClearCommand(LiftSubsystem liftSubsystem, IntakeSubsystem intake) {
        this.liftSubsystem = liftSubsystem;
        this.intake = intake;
        addRequirements(liftSubsystem);
    }
    @Override
    public void execute () {
        liftSubsystem.setSwingarmClearPosition();
    }
    @Override
    public boolean isFinished () {
        if (intake.getSkipLastSample()){
            return true;
        }else {
            return !liftSubsystem.isBusy();
        }
    }
    @Override
    public void end(boolean interrupted){
        liftSubsystem.stopLift();
    }
}
