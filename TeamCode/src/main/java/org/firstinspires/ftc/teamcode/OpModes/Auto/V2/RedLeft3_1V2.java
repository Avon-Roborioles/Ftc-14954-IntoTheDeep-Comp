package org.firstinspires.ftc.teamcode.OpModes.Auto.V2;



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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.OpModes.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoAfterScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoEndCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoToScore;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LeverCommands.LeverClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
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
public class RedLeft3_1V2 extends AutoBase {
    Command setPathToScorePreload, setPathToBar, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, setPark, setPathToBackAwayFromBar;
    Path toScorePreload, toBar, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark, park, backAwayFromBar;

    Pose Start = new Pose(- 7.8125 ,-60.8125, -PI/2);
    Pose Bar = new Pose(-7.5625, -32, -PI/2);
    Pose BackAwayFromBar = new Pose(-7.5625, -34, -PI/2);
    Pose BarMid = new Pose(-7.5625 , -38, -PI/2);
    Pose Score = new Pose(-57 ,-54 , PI/4 );
    Pose Grab1 = new Pose(-49.5, -41.5, PI/2);
    Pose Grab2 = new Pose(-60, -41.5, PI/2);
    Pose Grab3 = new Pose(-53, -34, 5* PI/6);
    Pose Grab3Mid = new Pose(-40, -36, 5* PI/6);
    Pose Park = new Pose(-24.6875, -11.0625, PI);
    Pose PrePark = new Pose(-33.1875, -11.0625, PI);
    Pose ParkMid = new Pose(-60, -11, PI/2);


    @Override
    public void initialize() {
        makeAuto();
        buildPaths();
        register(extend, liftSubsystem, swingArmSubsystem, pass, intake, box, wrist, autoDriveSubsystem);

        setPathToBar = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toBar, true);
        });
        setPathToScorePreload = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScorePreload, true);
        });
        setPathToBackAwayFromBar = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(backAwayFromBar, true);
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
        setPark = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(park, false);
        });

        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new LeverClearCommand(lever),
                new WaitCommand(100),
                new HandoffCommand(wrist),
                new RetractCommand(extend)
        );
        ParallelCommandGroup IntakeAndExtend =  new ParallelCommandGroup(
                new AutoIntake(intake, wrist),
                new LiftBottomCommand(liftSubsystem),
                new ExtensionCommand(extend, 0.50));

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                setPathToBar,
                new SwingArmDownCommand(swingArmSubsystem),
                new ParallelCommandGroup(
                        new LiftTopBarCommand(liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                initSubsystems,
                setPathToScorePreload,
                new ParallelCommandGroup(
                        new LiftTopBarCommand(liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToBackAwayFromBar,
                new ParallelCommandGroup(
                        new AutoClipSpecimen(liftSubsystem, 250),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new ExtensionCommand(extend, 0.64),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
                ),
                IntakeAndExtend,
                setPathToScore1,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp2,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem),
                        new ExtensionCommand(extend, 0.64),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                IntakeAndExtend,
                setPathToScore2,
                new ParallelCommandGroup(
                    new AutoToScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem),
                    new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToPickUp3,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem),
                        new ExtensionCommand(extend, 0.64),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                IntakeAndExtend,
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
        follower.setStartingPose(Start);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(Servo.class, "extension");
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"), hardwareMap.get(Rev2mDistanceSensor.class, "passDistance"), intake);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class,"boxDistance"), intake);
        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor1"),hardwareMap.get(ColorSensor.class, "intakeColor2"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), true, hardwareMap.get(CRServo.class, "intakeRoller"));
        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, Start);
        lever = new LeverSubsystem(hardwareMap.get(Servo.class, "lever"));
    }

    @Override
    public void buildPaths() {
        toBar = new Path(new BezierCurve(new Point(Start), new Point(BarMid)));
        toBar.setLinearHeadingInterpolation(Start.getHeading(), BarMid.getHeading());
        toBar.setPathEndTimeoutConstraint(250);

        toScorePreload = new Path(new BezierCurve(new Point(BarMid), new Point(Bar)));
        toScorePreload.setLinearHeadingInterpolation(BarMid.getHeading(), Bar.getHeading());
        toScorePreload.setPathEndTimeoutConstraint(500);

        backAwayFromBar = new Path(new BezierCurve(new Point(Bar), new Point(BackAwayFromBar)));
        backAwayFromBar.setLinearHeadingInterpolation(Bar.getHeading(), BackAwayFromBar.getHeading());
        backAwayFromBar.setPathEndTimeoutConstraint(250);

        toPickUp1 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(Grab1)));
        toPickUp1.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), Grab1.getHeading());
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
    }
}
