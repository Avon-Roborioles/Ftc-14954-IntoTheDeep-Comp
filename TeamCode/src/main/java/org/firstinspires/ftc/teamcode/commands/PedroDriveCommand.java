package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

import java.util.function.DoubleSupplier;

public class PedroDriveCommand extends CommandBase {
    private PedroDriveSubsystem pedroDriveSubsystem;
    private Telemetry telemetry;
    private double speed1 = 0;
    private DoubleSupplier strafe, forward, turn;
    private boolean fieldCentric;

    public PedroDriveCommand(PedroDriveSubsystem pedroDriveSubsystem, Telemetry telemetry, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, boolean fieldCentric){
        this.pedroDriveSubsystem = pedroDriveSubsystem;
        this.telemetry = telemetry;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.fieldCentric = fieldCentric;
        addRequirements(pedroDriveSubsystem);
    }
    @Override
    public void execute(){
        pedroDriveSubsystem.setTeleOpMovementVectors(forward.getAsDouble(), -strafe.getAsDouble(), -turn.getAsDouble(), fieldCentric);
        pedroDriveSubsystem.update();
    }

}
