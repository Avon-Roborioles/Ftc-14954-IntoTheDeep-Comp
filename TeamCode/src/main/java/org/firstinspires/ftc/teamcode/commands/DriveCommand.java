package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private DriveSubsystem drive;
    private DoubleSupplier strafe, forward, turn;
    public DriveCommand(DriveSubsystem drive, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn) {
        this.drive = drive;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        addRequirements(drive);
    }
    @Override
    public void execute() {
        drive.drive(strafe.getAsDouble() * 0.5, forward.getAsDouble()*0.5, turn.getAsDouble()*0.5);
    }
    @Override
    public boolean isFinished(){return false;}
}
