package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassOffCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.AutoSwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class AutoTopBucketScoreReady extends SequentialCommandGroup {
    public AutoTopBucketScoreReady(SwingArmSubsystem swingArm, LiftSubsystem lift, PassSubsystem pass, IntakeSubsystem intake){
        addCommands(
                new PassOffCommand(pass),
                new AutoLiftForSwingArmClearCommand(lift, intake),
                new AutoSwingArmMidCommand(swingArm, intake),
                new AutoLiftTopCommand(lift, intake)
        );
    }
}
