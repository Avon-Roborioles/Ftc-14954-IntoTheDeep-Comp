package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoSetStartCommand;
import org.firstinspires.ftc.teamcode.commands.VisionCommands.CameraAdjustCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@Autonomous
public class RedLeft extends  AutoBase {
    private Follower follower;

    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private Limelight3A limelight3A;
    private AutoDriveSubsystem autoDriveSubsystem;
    private AutoDriveCommand autoDriveCommand;
    private CameraAdjustCommand cameraAdjustCommand;
    private LimelightSubsystem limelightSubsystem;

    private TouchSensor touch1, touch2;
    private Servo extendservo;
    private Motor liftMotor;
    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private PassSubsystem pass;
    private IntakeSubsystem intake;
    private BoxxySubsystem box;
    private WristSubsystem wrist;

    Path toScan, toScorePreload, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark;
    Command setPathToScan, setPathToScorePreload, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, camera;

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
    public void initialize() {

    }
}
