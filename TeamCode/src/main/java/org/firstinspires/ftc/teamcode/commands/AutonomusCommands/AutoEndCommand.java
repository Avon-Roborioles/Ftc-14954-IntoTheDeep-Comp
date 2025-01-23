package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmParkCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoEndCommand extends SequentialCommandGroup {
    public AutoEndCommand(SwingArmSubsystem swingArm, LiftSubsystem lift){
        addCommands(
                new SwingArmParkCommand(swingArm),
                new LiftBottomCommand(lift),
                new SwingArmUpCommand(swingArm),
                new WaitCommand(250)
        );
    }
}
