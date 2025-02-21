package org.firstinspires.ftc.teamcode.commands.HangCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;

public class HangHoldCommand extends CommandBase {
    private HangSubsystem hangSubsystem;

    public HangHoldCommand(HangSubsystem hangSubsystem){
        this.hangSubsystem = hangSubsystem;
        addRequirements(hangSubsystem);
    }

    @Override
    public void execute(){
        hangSubsystem.setHangPower(-0.3);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
