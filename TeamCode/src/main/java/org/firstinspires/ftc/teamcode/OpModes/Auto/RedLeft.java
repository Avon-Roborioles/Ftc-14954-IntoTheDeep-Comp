package org.firstinspires.ftc.teamcode.OpModes.Auto;


import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoAfterScore;
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

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoToScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.PreloadToScore;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
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


    Path toScorePreload, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark, forward1, forward2, forward3;
    Command setPathToScorePreload, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, setPathToForward1, setPathToForward2, setPathToForward3;

    public static Pose RLStart = new Pose( 0,0, 0);
    public static Pose RLScore = new Pose(10, 17, -PI/4);
    public static Pose RLGrab1 = new Pose(22, 3, PI/6);
    public static Pose RLGrab2 = new Pose(22, 11, PI/6);
    public static Pose RLGrab3 = new Pose(24, 18, PI/4);
    public static Pose RLGrab3Mid = new Pose(19.5, 17, PI/4);
    public static Pose RLForward1 = new Pose(24, 5 , PI/6);
    public static Pose RLForward2 = new Pose(24, 13  , PI/6);
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
        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, RLStart);

        toScorePreload = new Path(new BezierCurve(new Point(RLStart), new Point(RLScore)));
        toScorePreload.setLinearHeadingInterpolation(RLStart.getHeading(), RLScore.getHeading());

        toPickUp1 = new Path(new BezierCurve(new Point(RLScore), new Point(RLGrab1)));
        toPickUp1.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab1.getHeading());

        toScore1 = new Path(new BezierCurve(new Point(RLGrab1), new Point(RLScore)));
        toScore1.setLinearHeadingInterpolation(RLGrab1.getHeading(), RLScore.getHeading());

        toPickUp2 = new Path(new BezierCurve(new Point(RLScore), new Point(RLGrab2)));
        toPickUp2.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab2.getHeading());

        toScore2 = new Path(new BezierCurve(new Point(RLGrab2), new Point(RLScore)));
        toScore2.setLinearHeadingInterpolation(RLGrab2.getHeading(), RLScore.getHeading());

        toPickUp3 = new Path(new BezierCurve(new Point(RLScore),new Point(RLGrab3Mid), new Point(RLGrab3)));
        toPickUp3.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab3.getHeading());

        toScore3 = new Path(new BezierCurve(new Point(RLGrab3), new Point(RLScore)));
        toScore3.setLinearHeadingInterpolation(RLGrab3.getHeading(), RLScore.getHeading());

        toPark = new Path(new BezierCurve(new Point(RLScore), new Point(RLPark)));
        toPark.setLinearHeadingInterpolation(RLScore.getHeading(), RLPark.getHeading());

        forward1 = new Path(new BezierCurve(new Point(RLGrab1), new Point(RLForward1)));
        forward1.setConstantHeadingInterpolation(RLGrab1.getHeading());

        forward2 = new Path(new BezierCurve(new Point(RLGrab2), new Point(RLForward2)));
        forward2.setConstantHeadingInterpolation(RLGrab2.getHeading());

        forward3 = new Path(new BezierCurve(new Point(RLGrab3), new Point(RLForward3)));
        forward3.setConstantHeadingInterpolation(RLGrab3.getHeading());


        register(extend, liftSubsystem, swingArmSubsystem, pass, intake, box, wrist, autoDriveSubsystem);


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
        ParallelCommandGroup IntakeAndDrive =  new ParallelCommandGroup(
                new AutoIntake(intake, wrist),
                new AutoDriveCommand(autoDriveSubsystem, telemetry));

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                initSubsystems,
                setPathToScorePreload,
                new ParallelCommandGroup(
                        new PreloadToScore(swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToForward1,
                IntakeAndDrive,
                setPathToScore1,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp2,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToForward2,
                IntakeAndDrive,
                setPathToScore2,
                new ParallelCommandGroup(
                    new AutoToScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                    new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp3,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToForward3,
                IntakeAndDrive,
                setPathToScore3,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                new AutoAfterScore(swingArmSubsystem, liftSubsystem)
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
