package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleSlowDriveCommand extends CommandBase {
    private AutoDriveSubsystem autoDriveSubsystem;
    private Telemetry telemetry;
    private double speed1 = 0.3;
    private DoubleSupplier strafe, forward, turn;
    private boolean fieldCentric;

    public TeleSlowDriveCommand(AutoDriveSubsystem autoDriveSubsystem, Telemetry telemetry, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, boolean fieldCentric){
        this.autoDriveSubsystem = autoDriveSubsystem;
        this.telemetry = telemetry;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.fieldCentric = fieldCentric;
        addRequirements(autoDriveSubsystem);
    }
    @Override
    public void execute(){
        autoDriveSubsystem.setTeleOpMovementVectors(forward.getAsDouble() * speed1, -strafe.getAsDouble() * speed1, -turn.getAsDouble() * speed1, fieldCentric);
        autoDriveSubsystem.update();
    }
}
