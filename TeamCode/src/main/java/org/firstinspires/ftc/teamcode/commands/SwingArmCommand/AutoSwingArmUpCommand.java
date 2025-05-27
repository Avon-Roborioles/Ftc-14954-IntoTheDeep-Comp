package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoSwingArmUpCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private NewIntakeSubsystem intake;

    public AutoSwingArmUpCommand(SwingArmSubsystem SwingArmSubsystem, NewIntakeSubsystem intake) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.intake = intake;
        addRequirements(SwingArmSubsystem);
    }

    @Override
    public void initialize() {
        if (!intake.getSkipLastSample()){
            SwingArmSubsystem.up();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
