package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.Storage.memory;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;


public class TelePathDriveCommand extends CommandBase {
    private AutoDriveSubsystem autoDriveSubsystem;
    private Pose pose;
    private Path path;

    public TelePathDriveCommand(AutoDriveSubsystem autoDriveSubsystem, Pose pose) {
        this.pose = pose;
        this.autoDriveSubsystem = autoDriveSubsystem;
        addRequirements(autoDriveSubsystem);
    }
    @Override
    public void initialize() {
        path = new Path(new BezierCurve(new Point(autoDriveSubsystem.getPose()), new Point(memory.scorePose)));
        path.setLinearHeadingInterpolation(autoDriveSubsystem.getPose().getHeading(), memory.scorePose.getHeading());
        autoDriveSubsystem.followPath(path, true);
    }
    @Override
    public void execute() {
        autoDriveSubsystem.update();
    }
    @Override
    public boolean isFinished() {
        return autoDriveSubsystem.atParametricEnd();
    }
    @Override
    public void end(boolean interrupted) {
        autoDriveSubsystem.breakFollowing();
        autoDriveSubsystem.startTeleopDrive();
    }

}
