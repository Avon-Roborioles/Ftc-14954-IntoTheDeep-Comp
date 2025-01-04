package org.firstinspires.ftc.teamcode.OpModes.Auto;


import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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

import org.firstinspires.ftc.teamcode.commands.VisionCommands.CameraAdjustCommand;
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
public class BlueLeft extends AutoBase{

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


    Path toScan, toScorePreload, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark;
    Command setPathToScan, setPathToScorePreload, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, camera;

    public static Pose BLStart = new Pose( 0,0, 3*PI/2);
    public static Pose BLScan = new Pose( 112.5,24.1, 3*PI/2);
    public static Pose BLScore = new Pose(14.9, 128.3, -3 * PI/4);
    public static Pose BLGrab1 = new Pose(24.1, 121, -PI/2);
    public static Pose BLGrab2 = new Pose(24.1, 131.7, -PI/2);
    public static Pose BLGrab3 = new Pose(45.7, 133.9, 0);
    public static Pose BLPark = new Pose(57.6, 96.3, PI);


    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(BLStart);

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
        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), false);
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSubsystem = new LimelightSubsystem(limelight3A, mTelemetry);

        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, BLStart);

        autoDriveCommand = new AutoDriveCommand(autoDriveSubsystem, mTelemetry);

        toScan = new Path(new BezierCurve(new Point(BLStart), new Point(BLScan)));
        toScan.setConstantHeadingInterpolation(BLScan.getHeading());
        toScan.setPathEndTimeoutConstraint(1500);

        toScorePreload = new Path(new BezierCurve(new Point(BLScan), new Point(BLScore)));
        toScorePreload.setLinearHeadingInterpolation(BLScan.getHeading(), BLScore.getHeading());
        toScorePreload.setPathEndTimeoutConstraint(1);

        toScore1 = new Path(new BezierCurve(new Point(BLGrab1), new Point(BLScore)));
        toScore1.setLinearHeadingInterpolation(BLGrab1.getHeading(), BLScore.getHeading());
        toScore1.setPathEndTimeoutConstraint(1);

        toPickUp2 = new Path(new BezierCurve(new Point(BLScore), new Point(BLGrab2)));
        toPickUp2.setLinearHeadingInterpolation(BLScore.getHeading(), BLGrab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(1);

        toScore2 = new Path(new BezierCurve(new Point(BLGrab2), new Point(BLScore)));
        toScore2.setLinearHeadingInterpolation(BLGrab2.getHeading(), BLScore.getHeading());
        toScore2.setPathEndTimeoutConstraint(1);

        toPickUp3 = new Path(new BezierCurve(new Point(BLScore), new Point(BLGrab3)));
        toPickUp3.setLinearHeadingInterpolation(BLScore.getHeading(), BLGrab3.getHeading());
        toPickUp3.setPathEndTimeoutConstraint(1);

        toScore3 = new Path(new BezierCurve(new Point(BLGrab3), new Point(BLScore)));
        toScore3.setLinearHeadingInterpolation(BLGrab3.getHeading(), BLScore.getHeading());
        toScore3.setPathEndTimeoutConstraint(1);

        toPark = new Path(new BezierCurve(new Point(BLScore), new Point(BLPark)));
        toPark.setLinearHeadingInterpolation(BLScore.getHeading(), BLPark.getHeading());
        toPark.setPathEndTimeoutConstraint(1);

        follower.followPath(toScan);


        register(extend, liftSubsystem, swingArmSubsystem, pass, intake, box, wrist, autoDriveSubsystem, limelightSubsystem);

        cameraAdjustCommand = new CameraAdjustCommand(autoDriveSubsystem, limelightSubsystem, mTelemetry);

        setPathToScan = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScan, true);
        });
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

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                setPathToScan, autoDriveCommand//,
//                        camera,
//                        setPathToScorePreload, autoDriveCommand,
//                        setPathToPickUp1, autoDriveCommand,
//                        setPathToScore1, autoDriveCommand,
//                        setPathToPickUp2, autoDriveCommand,
//                        setPathToScore2, autoDriveCommand,
//                        setPathToPickUp3, autoDriveCommand,
//                        setPathToScore3, autoDriveCommand,
//                        setPathToPark, autoDriveCommand
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
