package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftClearRampCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class PreloadToScore extends SequentialCommandGroup {

    public PreloadToScore(SwingArmSubsystem swingArm, LiftSubsystem lift, NewIntakeSubsystem intake, ClawSubsystem claw){
        addCommands(
                new LiftClearRampCommand(lift),
                new CloseClawCommand(claw),
                new LiftForSwingArmClearCommand(lift),
                new SwingArmUpCommand(swingArm),
                new LiftTopCommand(lift),
                new Score(swingArm, lift, claw, intake)
        );
    }
}
