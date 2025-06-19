package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

import java.util.concurrent.TimeUnit;

public class AutoSwingArmScoreCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private NewIntakeSubsystem intake;
    private ClawSubsystem claw;
    private Timing.Timer timer = new Timing.Timer(250, TimeUnit.MILLISECONDS);
    private Timing.Timer timer2 = new Timing.Timer(75, TimeUnit.MILLISECONDS);

    public AutoSwingArmScoreCommand(SwingArmSubsystem SwingArmSubsystem, NewIntakeSubsystem intake, ClawSubsystem claw) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.intake = intake;
        this.claw = claw;
        addRequirements(SwingArmSubsystem, intake, claw);
    }

    @Override
    public void initialize() {
        SwingArmSubsystem.up();

        timer.start();
        timer2.start();

    }
    @Override
    public void execute() {
        if (timer2.done()) {
            claw.open();
        }
    }

    @Override
    public boolean isFinished() {
        if (intake.getSkipLastSample()){
            return true;
        }else {
            return timer.done();
        }
    }
}