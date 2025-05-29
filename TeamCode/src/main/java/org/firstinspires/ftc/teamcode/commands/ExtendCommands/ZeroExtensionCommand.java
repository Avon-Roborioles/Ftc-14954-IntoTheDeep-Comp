package org.firstinspires.ftc.teamcode.commands.ExtendCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class ZeroExtensionCommand extends CommandBase {
    private ExtendSubsystem subsystem;

    public ZeroExtensionCommand(ExtendSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute () {
        subsystem.setPower(-1);
    }
    @Override
    public boolean isFinished () {
        return subsystem.retracted();
    }
    @Override
    public void end(boolean interrupted){
        subsystem.stop();
        subsystem.zero();
    }

}
