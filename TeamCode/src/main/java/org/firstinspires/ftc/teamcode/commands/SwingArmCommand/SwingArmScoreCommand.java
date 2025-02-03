package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;


import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

import java.util.concurrent.TimeUnit;

public class SwingArmScoreCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private BoxxySubsystem box;
    private IntakeSubsystem intake;
    private Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);

    public SwingArmScoreCommand(SwingArmSubsystem SwingArmSubsystem, BoxxySubsystem box, IntakeSubsystem intake) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.box = box;
        this.intake = intake;
        addRequirements(SwingArmSubsystem, box, intake);
    }

    @Override
    public void initialize() {
        SwingArmSubsystem.up();
        timer.start();
        if (box.IsBoxDistanceSensorCooked()) {
            intake.BoxFailLight();
        }


    }

    @Override
    public boolean isFinished() {
        if (box.IsBoxDistanceSensorCooked()) {
            return timer.done();
        } else {
            return !box.haveSample();
        }


    }
}