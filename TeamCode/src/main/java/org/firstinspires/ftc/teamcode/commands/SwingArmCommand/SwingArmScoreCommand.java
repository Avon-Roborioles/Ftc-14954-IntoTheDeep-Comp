package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

import java.util.concurrent.TimeUnit;

public class SwingArmScoreCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private BoxxySubsystem box;
    private Timing.Timer timer = new Timing.Timer(2, TimeUnit.SECONDS);

    public SwingArmScoreCommand(SwingArmSubsystem SwingArmSubsystem, BoxxySubsystem box) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.box = box;
        addRequirements(SwingArmSubsystem, box);
    }

    @Override
    public void initialize() {
        SwingArmSubsystem.up();
        timer.start();

    }

    @Override
    public boolean isFinished() {
        if (box.IsBoxDistanceSensorCooked()) {
            return timer.done();
        } else {
            return box.haveSample();
        }


    }
}