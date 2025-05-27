package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.AutoSwingArmUpCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoTopBucketScoreReady extends SequentialCommandGroup {
    public AutoTopBucketScoreReady(SwingArmSubsystem swingArm, LiftSubsystem lift, NewIntakeSubsystem intake){
        addCommands(

                new AutoLiftForSwingArmClearCommand(lift),
                new AutoSwingArmUpCommand(swingArm, intake),
                new AutoLiftTopCommand(lift, intake)
        );
    }
}
