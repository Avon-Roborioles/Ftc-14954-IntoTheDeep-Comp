package org.firstinspires.ftc.teamcode.commands.WristCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class WristClearBar extends CommandBase {
    private WristSubsystem subsystem;
    public WristClearBar(WristSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() { subsystem.ClearCenter();}
    @Override
    public boolean isFinished() { return true;}
}
