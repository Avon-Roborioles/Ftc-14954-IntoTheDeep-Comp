package org.firstinspires.ftc.teamcode.OpModes.Auto;


import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.commands.CommandGroups.IntakeToReadyForScore;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.VisionCommands.CameraAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class RedLeft extends AutoBase{

    private Follower follower;

    private Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    private AutoDriveSubsystem autoDriveSubsystem;
    private AutoDriveCommand autoDriveCommand;
    private Limelight3A limelight3A;
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


    Path toScan, toScorePreload, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark, forward1, forward2, forward3;
    Command setPathToScan, setPathToScorePreload, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, setPathToForward1, setPathToForward2, setPathToForward3;

    public static Pose RLStart = new Pose( 0,0, 0);
    public static Pose RLScore = new Pose(10, 17, -PI/4);
    public static Pose RLGrab1 = new Pose(22.5, 15, -PI/12);
    public static Pose RLGrab2 = new Pose(21.5, 23, 0);
    public static Pose RLGrab3 = new Pose(24, 18, PI/4);
    public static Pose RLGrab3Mid = new Pose(19.5, 17, PI/4);
    public static Pose RLForward1 = new Pose(24, 17, -PI/12);
    public static Pose RLForward2 = new Pose(26, 23, 0);
    public static Pose RLForward3 = new Pose(27, 21, PI/4);
    public static Pose RLPark = new Pose(0, 0, 0);


    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(RLStart);

        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(Servo.class, "extension");
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"));
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class,"boxDistance"));
        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), true);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSubsystem = new LimelightSubsystem(limelight3A, mTelemetry);

        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, RLStart);

        autoDriveCommand = new AutoDriveCommand(autoDriveSubsystem, mTelemetry);


        toScorePreload = new Path(new BezierCurve(new Point(RLStart), new Point(RLScore)));
        toScorePreload.setLinearHeadingInterpolation(RLStart.getHeading(), RLScore.getHeading());
        toScorePreload.setPathEndTimeoutConstraint(500);

        toPickUp1 = new Path(new BezierCurve(new Point(RLScore), new Point(RLGrab1)));
        toPickUp1.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(1000);

        toScore1 = new Path(new BezierCurve(new Point(RLGrab1), new Point(RLScore)));
        toScore1.setLinearHeadingInterpolation(RLGrab1.getHeading(), RLScore.getHeading());
        toScore1.setPathEndTimeoutConstraint(1000);

        toPickUp2 = new Path(new BezierCurve(new Point(RLScore), new Point(RLGrab2)));
        toPickUp2.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(1000);

        toScore2 = new Path(new BezierCurve(new Point(RLGrab2), new Point(RLScore)));
        toScore2.setLinearHeadingInterpolation(RLGrab2.getHeading(), RLScore.getHeading());
        toScore2.setPathEndTimeoutConstraint(1000);

        toPickUp3 = new Path(new BezierCurve(new Point(RLScore),new Point(RLGrab3Mid), new Point(RLGrab3)));
        toPickUp3.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab3.getHeading());
        toPickUp3.setPathEndTimeoutConstraint(1000);

        toScore3 = new Path(new BezierCurve(new Point(RLGrab3), new Point(RLScore)));
        toScore3.setLinearHeadingInterpolation(RLGrab3.getHeading(), RLScore.getHeading());
        toScore3.setPathEndTimeoutConstraint(1000);

        toPark = new Path(new BezierCurve(new Point(RLScore), new Point(RLPark)));
        toPark.setLinearHeadingInterpolation(RLScore.getHeading(), RLPark.getHeading());
        toPark.setPathEndTimeoutConstraint(1);

        forward1 = new Path(new BezierCurve(new Point(RLGrab1), new Point(RLForward1)));
        forward1.setConstantHeadingInterpolation(RLGrab1.getHeading());

        forward2 = new Path(new BezierCurve(new Point(RLGrab2), new Point(RLForward2)));
        forward2.setConstantHeadingInterpolation(RLGrab2.getHeading());

        forward3 = new Path(new BezierCurve(new Point(RLGrab3), new Point(RLForward3)));
        forward3.setConstantHeadingInterpolation(RLGrab3.getHeading());





        register(extend, liftSubsystem, swingArmSubsystem, pass, intake, box, wrist, autoDriveSubsystem, limelightSubsystem);

        cameraAdjustCommand = new CameraAdjustCommand(autoDriveSubsystem, limelightSubsystem, mTelemetry);

        setPathToScorePreload = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScorePreload, true);
        });
        setPathToPickUp1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPickUp1, true);
        });
        setPathToScore1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore1, true);
        });
        setPathToPickUp2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPickUp2, true);
        });
        setPathToScore2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore2, true);
        });
        setPathToPickUp3 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPickUp3, true);
        });
        setPathToScore3 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore3, true);
        });
        setPathToPark = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPark, false);
        });
        setPathToForward1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(forward1, true);
        });
        setPathToForward2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(forward2, true);
        });
        setPathToForward3 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(forward3, true);
        });

        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new HandoffCommand(wrist),
                new RetractCommand(extend)
        );

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                initSubsystems,
                setPathToScorePreload, autoDriveCommand,
                new LiftTopCommand(liftSubsystem),
                new Score(swingArmSubsystem, liftSubsystem, box),
                setPathToPickUp1, autoDriveCommand,
                setPathToForward1,
                new ParallelCommandGroup(
                new IntakeToReadyForScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToScore1, autoDriveCommand,
                new Score(swingArmSubsystem, liftSubsystem, box),
                setPathToPickUp2, autoDriveCommand,
                setPathToForward2,
                new ParallelCommandGroup(
                        new IntakeToReadyForScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToScore2, autoDriveCommand,
                new Score(swingArmSubsystem, liftSubsystem, box),
                setPathToPickUp3, autoDriveCommand,
                setPathToForward3,
                new ParallelCommandGroup(
                        new IntakeToReadyForScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToScore3, new AutoDriveCommand(autoDriveSubsystem, telemetry),
                new Score(swingArmSubsystem, liftSubsystem, box)





        );


        schedule(new SequentialCommandGroup(
                        number5IsAlive,
                        new InstantCommand(() -> {
                            follower.breakFollowing();
                            requestOpModeStop();
                        })
                )
        );
    }

    @Override
    public void makeAuto() {

    }

    @Override
    public void buildPaths() {

    }


}
