package org.firstinspires.ftc.teamcode.OpModes.Auto;




import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoAfterScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoEndCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoToScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.PreloadToScore;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LeverCommands.LeverClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LeverSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class BlueLeft4_0 extends AutoBase {

    Pose BLStartBucket = new Pose( 29.625 + 8.5625,70.125 - 8.1875, -PI/2);
    Pose BLScore = new Pose(57,54 , -3*PI/4 );
    Pose BLScorePreload = new Pose(55,55 , -3*PI/4 );
    Pose BLGrab1 = new Pose(49.5, 41, -PI/2);
    Pose BLForward1 = new Pose(49.5, 38, -PI/2);
    Pose BLGrab2 = new Pose(59.5, 41, -PI/2);
    Pose BLForward2 = new Pose(59.5, 38, -PI/2);
    Pose BLGrab3 = new Pose(53, 34, -PI/6);
    Pose BLGrab3Mid = new Pose(40, 36, -PI/6);
    Pose BLForward3 = new Pose(56, 33, -PI/6);
    Pose BLPark = new Pose(16.5 + 8.1875, 2.5 + 8.5625, 0);
    Pose BLPrePark = new Pose(25 + 8.1875, 2.5 + 8.5625, 0);
    Pose BLParkMid = new Pose(60, 11, -PI/2);






    @Override
    public void initialize() {
        makeAuto();
        buildPaths();
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
        setPark = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(park, false);
        });

        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new WaitCommand(10),
                new LeverClearCommand(lever),
                new WaitCommand(100),
                new HandoffCommand(wrist),
                new RetractCommand(extend)
        );
        ParallelCommandGroup IntakeAndDrive = new ParallelCommandGroup(
                new AutoIntake(intake, wrist),
                new LiftBottomCommand(liftSubsystem),
                new AutoDriveCommand(autoDriveSubsystem, telemetry));

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                initSubsystems,
                setPathToScorePreload,
                new ParallelCommandGroup(
                        new PreloadToScore(swingArmSubsystem, box, liftSubsystem, intake),
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
                setPathToPark,
                new ParallelCommandGroup(
                        new RaiseWrist(wrist),
                        new AutoEndCommand(swingArmSubsystem, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPark,
                new ParallelCommandGroup(
                        new AutoEndCommand(swingArmSubsystem, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry))
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
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(BLStartBucket);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(Servo.class, "extension");
        extend = new ExtendSubsystem(extendservo, touch2);
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor, touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"), hardwareMap.get(Rev2mDistanceSensor.class, "passDistance"), intake);
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class, "boxDistance"), intake);
        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor1"), hardwareMap.get(ColorSensor.class, "intakeColor2"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), false, hardwareMap.get(CRServo.class, "intakeRoller"));
        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, BLStartBucket);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        lever = new LeverSubsystem(hardwareMap.get(Servo.class, "lever"));
    }

    @Override
    public void buildPaths() {
        toScorePreload = new Path(new BezierCurve(new Point(BLStartBucket), new Point(BLScorePreload)));
        toScorePreload.setLinearHeadingInterpolation(BLStartBucket.getHeading(), BLScorePreload.getHeading());
        toScorePreload.setPathEndTimeoutConstraint(1000);

        toPickUp1 = new Path(new BezierCurve(new Point(BLScorePreload), new Point(BLGrab1)));
        toPickUp1.setLinearHeadingInterpolation(BLScorePreload.getHeading(), BLGrab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(1000);

        toScore1 = new Path(new BezierCurve(new Point(BLGrab1), new Point(BLScore)));
        toScore1.setLinearHeadingInterpolation(BLGrab1.getHeading(), BLScore.getHeading());
        toScore1.setPathEndTimeoutConstraint(2000);

        toPickUp2 = new Path(new BezierCurve(new Point(BLScore), new Point(BLGrab2)));
        toPickUp2.setLinearHeadingInterpolation(BLScore.getHeading(), BLGrab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(1000);

        toScore2 = new Path(new BezierCurve(new Point(BLGrab2), new Point(BLScore)));
        toScore2.setLinearHeadingInterpolation(BLGrab2.getHeading(), BLScore.getHeading());
        toScore2.setPathEndTimeoutConstraint(1000);

        toPickUp3 = new Path(new BezierCurve(new Point(BLScore), new Point(BLGrab3Mid), new Point(BLGrab3)));
        toPickUp3.setLinearHeadingInterpolation(BLScore.getHeading(), BLGrab3.getHeading());
        toPickUp3.setPathEndTimeoutConstraint(1000);

        toScore3 = new Path(new BezierCurve(new Point(BLGrab3), new Point(BLScore)));
        toScore3.setLinearHeadingInterpolation(BLGrab3.getHeading(), BLScore.getHeading());
        toScore3.setPathEndTimeoutConstraint(1000);

        toPark = new Path(new BezierCurve(new Point(BLScore), new Point(BLParkMid), new Point(BLPrePark)));
        toPark.setLinearHeadingInterpolation(BLScore.getHeading(), BLPrePark.getHeading());
        toPark.setPathEndTimeoutConstraint(1000);

        park = new Path(new BezierCurve(new Point(BLPrePark), new Point(BLPark)));
        park.setLinearHeadingInterpolation(BLPrePark.getHeading(), BLPark.getHeading());
        park.setPathEndTimeoutConstraint(250);

        forward1 = new Path(new BezierCurve(new Point(BLGrab1), new Point(BLForward1)));
        forward1.setConstantHeadingInterpolation(BLGrab1.getHeading());

        forward2 = new Path(new BezierCurve(new Point(BLGrab2), new Point(BLForward2)));
        forward2.setConstantHeadingInterpolation(BLGrab2.getHeading());

        forward3 = new Path(new BezierCurve(new Point(BLGrab3), new Point(BLForward3)));
        forward3.setConstantHeadingInterpolation(BLGrab3.getHeading());
    }
}
