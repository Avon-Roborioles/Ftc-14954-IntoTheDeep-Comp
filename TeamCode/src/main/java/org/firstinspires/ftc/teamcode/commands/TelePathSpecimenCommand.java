package org.firstinspires.ftc.teamcode.commands;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.Storage.memory;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;


public class TelePathSpecimenCommand extends CommandBase {
    private AutoDriveSubsystem autoDriveSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private Follower follower;
    private Pose pose;
    private Path path;

    public TelePathSpecimenCommand(AutoDriveSubsystem autoDriveSubsystem, Pose pose, Follower follower, LimelightSubsystem limelightSubsystem) {
        this.pose = pose;
        this.autoDriveSubsystem = autoDriveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.follower = follower;
        addRequirements(autoDriveSubsystem);
    }
    @Override
    public void initialize() {
        path = new Path(new BezierCurve(new Point(autoDriveSubsystem.getPose()), new Point(new Pose(limelightSubsystem.getStrafe(follower), -60, PI/2))));
        path.setLinearHeadingInterpolation(autoDriveSubsystem.getPose().getHeading(), PI/2);
        autoDriveSubsystem.setMaxPower(0.5);
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
