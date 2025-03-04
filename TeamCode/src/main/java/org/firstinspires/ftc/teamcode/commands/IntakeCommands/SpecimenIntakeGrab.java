package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SpecimenIntakeGrab extends CommandBase {
    private IntakeSubsystem intake;
    private WristSubsystem wrist;
    public SpecimenIntakeGrab(IntakeSubsystem intake, WristSubsystem wrist) {
        this.intake = intake;
        this.wrist = wrist;
        addRequirements(intake, wrist);
    }
    @Override
        public void execute() {
            intake.runMotor();
            wrist.down();
        }
        public void end(boolean interrupted) {
            intake.stopMotor();
            wrist.up();
        }
        public boolean isFinished() {
            return intake.hasSample();
        }
    }


