package org.firstinspires.ftc.teamcode.commands.CommandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class Score extends SequentialCommandGroup {
    private SwingArmSubsystem swingArm;
    private LiftSubsystem lift;
    private BoxxySubsystem box;
    public Score(SwingArmSubsystem swingArm, LiftSubsystem lift){
        addCommands(
                new LiftTopCommand(lift),
                new SwingArmUpCommand(swingArm, box),
                new WaitCommand(1000),
                new SwingArmDownCommand(swingArm),
                new WaitCommand(500),
                new LiftBottomCommand(lift)
        );
    }
}
