package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;


import org.firstinspires.ftc.teamcode.commands.CommandGroups.NewScore;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

import java.util.concurrent.TimeUnit;

public class SwingArmScoreCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private BoxxySubsystem box;
    private NewIntakeSubsystem intake;
    private Timing.Timer timer = new Timing.Timer(1, TimeUnit.SECONDS);

    public SwingArmScoreCommand(SwingArmSubsystem SwingArmSubsystem, NewIntakeSubsystem intake) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.intake = intake;
        addRequirements(SwingArmSubsystem, intake);
    }

    @Override
    public void initialize() {
        SwingArmSubsystem.up();
//        timer.start();



    }

    @Override
    public boolean isFinished() {
        return true;
//        if (box.IsBoxDistanceSensorCooked()) {
//            return timer.done();
//        } else {
//            return !box.haveSample();
//        }


    }
}