package org.firstinspires.ftc.teamcode.commands.ClawCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class OpenClawCommand extends CommandBase {
    private ClawSubsystem claw;
    public OpenClawCommand(ClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }
    @Override
    public void initialize() { claw.open(); }
    @Override
    public boolean isFinished() { return true; }
}
