package org.firstinspires.ftc.teamcode.commands.ClawCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class CloseClawCommand extends CommandBase {
    private ClawSubsystem claw;
    public CloseClawCommand(ClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }
    @Override
    public void initialize() { claw.close(); }
    @Override
    public boolean isFinished() { return true; }
}
