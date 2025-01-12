package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class SwingArmParkCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;

    public SwingArmParkCommand(SwingArmSubsystem SwingArmSubsystem) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        addRequirements(SwingArmSubsystem);
    }

    @Override
    public void initialize() {
        SwingArmSubsystem.park();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
