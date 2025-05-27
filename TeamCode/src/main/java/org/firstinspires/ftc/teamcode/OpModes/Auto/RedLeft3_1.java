package org.firstinspires.ftc.teamcode.OpModes.Auto;



import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoAfterScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoEndCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoToScore;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.ClipTopSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import pedroPathing.constants.FConstantsL;
import pedroPathing.constants.LConstants;
@Disabled
@Autonomous
public class RedLeft3_1 extends AutoBase{
    Command setPathToScorePreload, setPathToBar, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, setPark, setPathToForward1, setPathToForward2, setPathToForward3;
    Path toScorePreload, toBar, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark, park, forward1, forward2, forward3;

    Pose Start = new Pose(- 7.8125 ,-60.8125, -PI/2);
    Pose Bar = new Pose(-7.5625, -32, -PI/2);
    Pose BarMid = new Pose(-7.5625 , -38, -PI/2);
    Pose Score = new Pose(-57 ,-54 , PI/4 );
    Pose Grab1 = new Pose(-49.5, -41.5, PI/2);
    Pose Grab1Mid = new Pose(-40, -50, PI/2);
    Pose Forward1 = new Pose(-49.5, -38, PI/2);
    Pose Grab2 = new Pose(-60, -41.5, PI/2);
    Pose Forward2 = new Pose(-60, -38, PI/2);
    Pose Grab3 = new Pose(-53, -34, 5* PI/6);
    Pose Grab3Mid = new Pose(-40, -36, 5* PI/6);
    Pose Forward3 = new Pose(-56, -33, 5* PI/6);
    Pose Park = new Pose(-24.6875, -11.0625, PI);
    Pose PrePark = new Pose(-33.1875, -11.0625, PI);
    Pose ParkMid = new Pose(-60, -11, PI/2);


    @Override
    public void initialize() {
        makeAuto();
        buildPaths();
        register(extend, liftSubsystem, swingArmSubsystem,  intake, wrist, autoDriveSubsystem, claw);

        setPathToBar = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toBar, true);
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
                new WaitCommand(100),
                new RaiseWrist(wrist),
                new RetractCommand(extend),
                new InstantCommand(() -> {
                    Storage.memory.scorePose = Score;
                })
        );
        ParallelCommandGroup IntakeAndDrive =  new ParallelCommandGroup(
                new AutoIntake(intake, wrist),
                new LiftBottomCommand(liftSubsystem),
                new AutoDriveCommand(autoDriveSubsystem, telemetry));

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                new SwingArmDownCommand(swingArmSubsystem),
                setPathToBar,
                new ParallelCommandGroup(
                        new LiftTopBarCommand(liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                initSubsystems,
                setPathToScorePreload,
                new ParallelCommandGroup(
                        new LiftTopBarCommand(liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                new LiftForSwingArmClearCommand(liftSubsystem),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new ClipTopSpecimen(liftSubsystem, 1000),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry))),
                setPathToForward1,
                IntakeAndDrive,
                setPathToScore1,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp2,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem, claw),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToForward2,
                IntakeAndDrive,
                setPathToScore2,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw),
                    new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp3,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem, claw),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToForward3,
                IntakeAndDrive,
                setPathToScore3,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw),
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
        Constants.setConstants(FConstantsL.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(Start);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(ServoImplEx.class, "extension");
        extendservo.setPwmRange(servoRange);
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        claw = new ClawSubsystem(hardwareMap.get(Servo.class, "claw"));
        intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"), false);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, Start);

    }

    @Override
    public void buildPaths() {
        toBar = new Path(new BezierCurve(new Point(Start), new Point(BarMid)));
        toBar.setLinearHeadingInterpolation(Start.getHeading(), BarMid.getHeading());
        toBar.setPathEndTimeoutConstraint(250);

        toScorePreload = new Path(new BezierCurve(new Point(BarMid), new Point(Bar)));
        toScorePreload.setLinearHeadingInterpolation(BarMid.getHeading(), Bar.getHeading());


        toPickUp1 = new Path(new BezierCurve(new Point(Score),new Point(Grab1Mid), new Point(Grab1)));
        toPickUp1.setLinearHeadingInterpolation(Score.getHeading(), Grab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(500);

        toScore1 = new Path(new BezierCurve(new Point(Grab1), new Point(Score)));
        toScore1.setLinearHeadingInterpolation(Grab1.getHeading(), Score.getHeading());
        toScore1.setPathEndTimeoutConstraint(2000);

        toPickUp2 = new Path(new BezierCurve(new Point(Score), new Point(Grab2)));
        toPickUp2.setLinearHeadingInterpolation(Score.getHeading(), Grab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(1000);

        toScore2 = new Path(new BezierCurve(new Point(Grab2), new Point(Score)));
        toScore2.setLinearHeadingInterpolation(Grab2.getHeading(), Score.getHeading());
        toScore2.setPathEndTimeoutConstraint(1000);

        toPickUp3 = new Path(new BezierCurve(new Point(Score),new Point(Grab3Mid), new Point(Grab3)));
        toPickUp3.setLinearHeadingInterpolation(Score.getHeading(), Grab3.getHeading());
        toPickUp3.setPathEndTimeoutConstraint(1000);

        toScore3 = new Path(new BezierCurve(new Point(Grab3), new Point(Score)));
        toScore3.setLinearHeadingInterpolation(Grab3.getHeading(), Score.getHeading());
        toScore3.setPathEndTimeoutConstraint(1000);

        toPark = new Path(new BezierCurve(new Point(Score),new Point(ParkMid), new Point(PrePark)));
        toPark.setLinearHeadingInterpolation(Score.getHeading(), PrePark.getHeading());
        toPark.setPathEndTimeoutConstraint(1000);

        park= new Path(new BezierCurve(new Point(PrePark), new Point(Park)));
        park.setLinearHeadingInterpolation(PrePark.getHeading(), Park.getHeading());
        park.setPathEndTimeoutConstraint(250);

        forward1 = new Path(new BezierCurve(new Point(Grab1), new Point(Forward1)));
        forward1.setConstantHeadingInterpolation(Grab1.getHeading());

        forward2 = new Path(new BezierCurve(new Point(Grab2), new Point(Forward2)));
        forward2.setConstantHeadingInterpolation(Grab2.getHeading());

        forward3 = new Path(new BezierCurve(new Point(Grab3), new Point(Forward3)));
        forward3.setConstantHeadingInterpolation(Grab3.getHeading());
    }


}
