package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class SwingArmScoreCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    private BoxxySubsystem box;
    public SwingArmScoreCommand(SwingArmSubsystem SwingArmSubsystem, BoxxySubsystem box) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        this.box = box;
        addRequirements(SwingArmSubsystem, box);
    }
    @Override
    public void initialize() { SwingArmSubsystem.up(); }

    @Override
    public boolean isFinished() { return box.noSample(); }
}