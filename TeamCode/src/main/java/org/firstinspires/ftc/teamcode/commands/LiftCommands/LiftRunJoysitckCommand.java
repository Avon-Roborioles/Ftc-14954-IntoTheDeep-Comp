package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import java.util.function.DoubleSupplier;

public class LiftRunJoysitckCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    private DoubleSupplier joystickInput;

    public LiftRunJoysitckCommand(LiftSubsystem liftSubsystem, DoubleSupplier joystickInput) {
        this.liftSubsystem = liftSubsystem;
        this.joystickInput = joystickInput;
        addRequirements(liftSubsystem);
    }

    @Override
    public void execute() {
        liftSubsystem.runLiftJoystick(joystickInput.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.stopLift();
    }

}
