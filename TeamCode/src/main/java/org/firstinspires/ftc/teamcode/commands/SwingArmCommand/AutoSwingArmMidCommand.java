package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoSwingArmMidCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private NewIntakeSubsystem intake;

    public AutoSwingArmMidCommand(SwingArmSubsystem SwingArmSubsystem, NewIntakeSubsystem intake) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.intake = intake;
        addRequirements(SwingArmSubsystem);
    }

    @Override
    public void initialize() {
        if (!intake.getSkipLastSample()) {
            SwingArmSubsystem.mid();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
