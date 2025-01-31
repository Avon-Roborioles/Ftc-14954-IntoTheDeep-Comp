package org.firstinspires.ftc.teamcode.commands.HangCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;

import java.util.function.DoubleSupplier;

public class HangJoystickCommand extends CommandBase {
    private HangSubsystem hangSubsystem;
    private DoubleSupplier power;
    public HangJoystickCommand(HangSubsystem hangSubsystem, DoubleSupplier power) {
        this.hangSubsystem = hangSubsystem;
        this.power = power;
        addRequirements(hangSubsystem);
    }
    @Override
    public void execute() {
        hangSubsystem.setHangPower(power.getAsDouble());
    }
    @Override
    public boolean isFinished(){return false;}


}
