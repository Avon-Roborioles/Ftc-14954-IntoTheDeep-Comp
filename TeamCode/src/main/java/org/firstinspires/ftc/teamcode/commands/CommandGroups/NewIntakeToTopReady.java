package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ClawCommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NomNomComand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class NewIntakeToTopReady extends SequentialCommandGroup {
    private NewIntakeSubsystem intake;
    private WristSubsystem wrist;
    private ClawSubsystem claw;
    private LiftSubsystem lift;
    private ExtendSubsystem extend;
    private SwingArmSubsystem swingArm;
    private Telemetry telemetry;
    public NewIntakeToTopReady (NewIntakeSubsystem intake, WristSubsystem wrist, ClawSubsystem claw, LiftSubsystem lift, ExtendSubsystem extend, SwingArmSubsystem swingArm){
        addCommands(
                new NomNomComand(intake, wrist, extend,telemetry, false),
                new CloseClawCommand(claw),
                new LiftTopCommand(lift),
                new SwingArmMidCommand(swingArm)
        );
    }
}
