package org.firstinspires.ftc.teamcode.OpModes.Auto.Left;



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
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.OpModes.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoAfterScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoEndCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoToScore;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ZeroExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


public class Left4_1 extends AutoBase {
    Command setPathToScorePreload, setPathToBar, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, setPark, setPathToBackAwayFromBar;
    Path toScorePreload, toBar, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark, park, backAwayFromBar;

    Pose Start = new Pose(- 7.8125 ,-61.5, -PI/2);
    Pose Bar = new Pose(-7.5625, -31.5, -PI/2);
    Pose BackAwayFromBar = new Pose(-7.5625, -34, -PI/2);
    Pose BarMid = new Pose(-7.5625 , -38, -PI/2);
    Pose Score = new Pose(-56 ,-52.5 , PI/4 );
    Pose Grab1 = new Pose(-48.5, -43, PI/2);
    Pose Grab2 = new Pose(-59, -43, PI/2);
    Pose Grab3 = new Pose(-51, -35.25, 5* PI/6);
    Pose Grab3Mid = new Pose(-40, -36, 5* PI/6);
    Pose Grab4 = new Pose(-25, -11.0625, 0);
    Pose Grab4Mid = new Pose(-60, -11, PI/2);
    Pose Strafe = new Pose(-25, -7, 0);

    Boolean RedAlliance = null;

    @Override
    public void initialize() {
        RedAlliance = null;
        SetAllianceColor();
        makeAuto();
        buildPaths();
        register(extend, liftSubsystem, swingArmSubsystem,  intake, wrist, autoDriveSubsystem, claw);

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
        setPathToGrab4 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toGrab4, true);
        });
        setPathToScore4 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore4, true);
        });
        setPathToStrafe = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(strafe, true);
        });


        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new WaitCommand(100),
                new RaiseWrist(wrist),
                new ZeroExtensionCommand(extend),
                new InstantCommand(() -> {
                    Storage.memory.scorePose = Score;
                })
        );
        ParallelCommandGroup IntakeAndExtend =  new ParallelCommandGroup(
                new ExtensionCommand(extend, 0.75),
                new AutoIntake(intake, wrist),
                new LiftBottomCommand(liftSubsystem)
        );

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
                        new ExtensionCommand(extend, 0.65),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
                ),
                IntakeAndExtend,
                setPathToScore1,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                new AutoScore(intake, swingArmSubsystem,claw),
                setPathToPickUp2,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem, claw),
                        new ExtensionCommand(extend, 0.65),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                IntakeAndExtend,
                setPathToScore2,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                new AutoScore(intake, swingArmSubsystem,claw),
                setPathToPickUp3,
                new ParallelCommandGroup(
                        new AutoAfterScore(swingArmSubsystem, liftSubsystem, claw),
                        new ExtensionCommand(extend, 0.65),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                IntakeAndExtend,
                setPathToScore3,
                new ParallelCommandGroup(
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                new AutoScore(intake, swingArmSubsystem,claw),
                new AutoAfterScore(swingArmSubsystem, liftSubsystem, claw, extend),
                setPathToGrab4,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new ExtensionCommand(extend, 0.5)
                ),
                new LowerWrist(wrist),
                setPathToStrafe,
                new ParallelCommandGroup(
                        new CollectSample(intake),
                        new ExtensionCommand(extend, 0.8),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
                ),
                setPathToScore4,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new AutoToScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw)
                ),
                new AutoScore(intake, swingArmSubsystem,claw),
                new SwingArmDownCommand(swingArmSubsystem),
                new LiftForSwingArmClearCommand(liftSubsystem),
                new LiftBottomCommand(liftSubsystem)
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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(Start);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extend = new ExtendSubsystem(touch2, hardwareMap.get(DcMotorEx.class, "extensionMotor"));
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        claw = new ClawSubsystem(hardwareMap.get(Servo.class, "claw"));
        if (RedAlliance != null){
            intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"), RedAlliance);
        }else {
            intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"));
        }
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
        toScorePreload.setPathEndTimeoutConstraint(250);

        backAwayFromBar = new Path(new BezierCurve(new Point(Bar), new Point(BackAwayFromBar)));
        backAwayFromBar.setLinearHeadingInterpolation(Bar.getHeading(), BackAwayFromBar.getHeading());
        backAwayFromBar.setPathEndTimeoutConstraint(50);

        toPickUp1 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(Grab1)));
        toPickUp1.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), Grab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(500);

        toScore1 = new Path(new BezierCurve(new Point(Grab1), new Point(Score)));
        toScore1.setLinearHeadingInterpolation(Grab1.getHeading(), Score.getHeading());
        toScore1.setPathEndTimeoutConstraint(750);

        toPickUp2 = new Path(new BezierCurve(new Point(Score), new Point(Grab2)));
        toPickUp2.setLinearHeadingInterpolation(Score.getHeading(), Grab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(750);

        toScore2 = new Path(new BezierCurve(new Point(Grab2), new Point(Score)));
        toScore2.setLinearHeadingInterpolation(Grab2.getHeading(), Score.getHeading());
        toScore2.setPathEndTimeoutConstraint(750);

        toPickUp3 = new Path(new BezierCurve(new Point(Score),new Point(Grab3Mid), new Point(Grab3)));
        toPickUp3.setLinearHeadingInterpolation(Score.getHeading(), Grab3.getHeading());
        toPickUp3.setPathEndTimeoutConstraint(750);

        toScore3 = new Path(new BezierCurve(new Point(Grab3), new Point(Score)));
        toScore3.setLinearHeadingInterpolation(Grab3.getHeading(), Score.getHeading());
        toScore3.setPathEndTimeoutConstraint(750);

        toGrab4 = new Path(new BezierCurve(new Point(Score), new Point(Grab4Mid), new Point(Grab4)));
        toGrab4.setLinearHeadingInterpolation(Score.getHeading(), Grab4.getHeading());
        toGrab4.setPathEndTimeoutConstraint(750);

        toScore4 = new Path(new BezierCurve(new Point(Grab4), new Point(Grab4Mid), new Point(Score)));
        toScore4.setLinearHeadingInterpolation(Grab4.getHeading(), Score.getHeading());
        toScore4.setPathEndTimeoutConstraint(750);

        strafe = new Path(new BezierCurve(new Point(Grab4), new Point(Strafe)));
        strafe.setLinearHeadingInterpolation(Grab4.getHeading(), Strafe.getHeading());
        strafe.setPathEndTimeoutConstraint(50);
    }
    public void SetAllianceColor(){
    }
}
