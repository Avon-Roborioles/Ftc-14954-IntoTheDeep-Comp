package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoSetStartCommand;
import org.firstinspires.ftc.teamcode.commands.VisionCommands.CameraAdjustCommand;
import org.firstinspires.ftc.teamcode.oldPedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.oldPedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.oldPedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.oldPedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.oldPedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@Autonomous
public class RedLeft extends  AutoBase {
    @Override
    public void makeAuto() {
        //        old Code
        Pose scanPose = new Pose(-40.75, -50, 0);
        Pose startPose = new Pose(-40.75, -64.5, 0);
        Pose scorePose = new Pose(-55, -55, PI / 4);
        Pose firstPose = new Pose(-48, -32, PI / 2);
        Pose secondPose = new Pose(-55, -26, PI / 2);
        Pose thirdPose = new Pose(-55, -32, PI);

        Path toScan;
        Path toFirst;
        Path fromFirst;
        Path toSecond;
        Path fromSecond;
        Path toThird;
        Path fromThird;


        toScan = new Path((new BezierCurve(new Point(startPose), new Point(scanPose))));
        toScan.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());
        toScan.setPathEndTimeoutConstraint(1500);

        toFirst = new Path((new BezierCurve(new Point(scanPose), new Point(firstPose))));
        toFirst.setLinearHeadingInterpolation(scorePose.getHeading(), firstPose.getHeading());
        toFirst.setPathEndTimeoutConstraint(3000);
        fromFirst = new Path((new BezierCurve(new Point(firstPose), new Point(scorePose))));
        fromFirst.setLinearHeadingInterpolation(firstPose.getHeading(), scorePose.getHeading());
        fromFirst.setPathEndTimeoutConstraint(1000);
        toSecond = new Path((new BezierCurve(new Point(scorePose), new Point(secondPose))));
        toSecond.setLinearHeadingInterpolation(scorePose.getHeading(), secondPose.getHeading());
        toSecond.setPathEndTimeoutConstraint(3000);
        fromSecond = new Path((new BezierCurve(new Point(secondPose), new Point(scorePose))));
        fromSecond.setLinearHeadingInterpolation(secondPose.getHeading(), scorePose.getHeading());
        fromSecond.setPathEndTimeoutConstraint(3000);
        toThird = new Path((new BezierCurve(new Point(scorePose), new Point(thirdPose), new Point(new Pose(-60, -32, PI)))));
        toThird.setLinearHeadingInterpolation(scorePose.getHeading(), thirdPose.getHeading());
        toThird.setPathEndTimeoutConstraint(3000);
        fromThird = new Path((new BezierCurve(new Point(thirdPose), new Point(scorePose))));
        fromThird.setLinearHeadingInterpolation(thirdPose.getHeading(), scorePose.getHeading());
        fromThird.setPathEndTimeoutConstraint(3000);


        follower = new Follower(hardwareMap);

        AutoSetStartCommand autoSetStartCommand = new AutoSetStartCommand(startPose, follower);

        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, startPose);
        autoDriveSubsystem.setMaxPower(1);

        autoDriveCommand = new AutoDriveCommand(autoDriveSubsystem, mTelemetry);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSubsystem = new LimelightSubsystem(limelight3A, mTelemetry);
        cameraAdjustCommand = new CameraAdjustCommand(autoDriveSubsystem, limelightSubsystem, mTelemetry);


        register(autoDriveSubsystem);
        Command setPathToScan = new InstantCommand(() -> {
            autoDriveSubsystem.setMaxPower(0.5);
            autoDriveSubsystem.followPath(toScan, true);
        });
        Command setPathToFirst = new InstantCommand(() -> {
            autoDriveSubsystem.setMaxPower(0.5);
            autoDriveSubsystem.followPath(toFirst, true);
        });
        Command setPathFromFirst = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(fromFirst, true);
            autoDriveSubsystem.setMaxPower(1);
        });
        Command setPathToSecond = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toSecond, true);
        });
        Command setPathFromSecond = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(fromSecond, true);
        });
        Command setPathToThird = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toThird, true);
        });
        Command setPathFromThird = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(fromThird, true);
        });

        Command camera = new ParallelCommandGroup(new WaitCommand(1000), cameraAdjustCommand);


        schedule(new SequentialCommandGroup(
                autoSetStartCommand,
                setPathToScan, autoDriveCommand,
                camera,
                setPathToFirst, autoDriveCommand,
                setPathFromFirst, autoDriveCommand,
                setPathToSecond, autoDriveCommand,
                setPathFromSecond, autoDriveCommand,
                setPathToThird, autoDriveCommand,
                setPathFromThird, autoDriveCommand
        ));
    }

    @Override
    public void buildPaths() {

    }

    @Override
    public void buildCommands() {

    }

    @Override
    public void initialize() {

    }
}
