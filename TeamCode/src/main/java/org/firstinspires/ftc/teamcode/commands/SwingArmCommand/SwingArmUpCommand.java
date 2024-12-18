package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class SwingArmUpCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private BoxxySubsystem box;
    public SwingArmUpCommand(SwingArmSubsystem SwingArmSubsystem, BoxxySubsystem box) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.box = box;
        addRequirements(SwingArmSubsystem, box);
    }
    @Override
    public void initialize() { SwingArmSubsystem.up(); }

    @Override
    public boolean isFinished() { return box.noSample(); }
}