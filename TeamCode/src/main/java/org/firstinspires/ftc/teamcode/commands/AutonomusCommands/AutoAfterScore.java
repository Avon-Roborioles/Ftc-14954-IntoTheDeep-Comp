package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftClearRampCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoAfterScore extends SequentialCommandGroup {
    public AutoAfterScore(SwingArmSubsystem swingArm, LiftSubsystem lift, ClawSubsystem claw){
        addCommands(
                new SwingArmMidCommand(swingArm),
                new LiftForSwingArmClearCommand(lift),
                new SwingArmDownCommand(swingArm),
                new LiftClearRampCommand(lift),
                new OpenClawCommand(claw)
        );
    }
}
