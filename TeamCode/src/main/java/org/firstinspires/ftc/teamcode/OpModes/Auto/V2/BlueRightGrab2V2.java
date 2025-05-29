package org.firstinspires.ftc.teamcode.OpModes.Auto.V2;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.OpModes.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoClipSpecimen;
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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class BlueRightGrab2V2 extends AutoBase {
    public PathChain run1;
    public Pose Start = new Pose( 7.8125 ,-60.0125, -PI/2);
    public Pose Bar = new Pose( 8.25 , -31.25, -PI/2);
    public Pose BarMid = new Pose( 8.25 , -32.5, -PI/2);
    public Pose BackAwayFromBar = new Pose( 8.5625-0.75 , -33, -PI/2);
    public Pose StrafeOver = new Pose(30, -38, PI/2);
    public Pose Clear1 = new Pose(32, -17,PI/2);
    public Pose Over1 = new Pose(41, -17,PI/2);
    public Pose Push1 = new Pose(41, -50, PI/2);
    public Pose Over2 = new Pose(53, -16, PI/2);
    public Pose Push2 = new Pose(42, -56, PI/2);
    public Pose StrafeOver2 = new Pose(35, -56, PI/2);

    public Pose Grab2 = new Pose(32, -38, -PI/2);

    public Pose ForSpecimen = new Pose(30.5, -58, PI/2);
    public Pose GrabSpecimen = new Pose(ForSpecimen.getX(),Start.getY()-2, PI/2);



    public Pose Park = new Pose(60, -60, PI);
    public Pose ParkMid = new Pose(40, -40, PI);

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
        setPathToBackAwayFromBar = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(backAwayFromBar, true);
        });
        setPathToPickUp1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(run1, true);
        });
        setPathToPickUp2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPickUp2, true);
        });
        setPathToToSpecimen1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toSpecimen1, true);
        });
        setPathToGrabSpecimen = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toGrabSpecimen, true);
            autoDriveSubsystem.setMaxPower(1);
        });
        setPathToScore1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore1, true);
            autoDriveSubsystem.setMaxPower(1);
        });
        setPathToForward1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(forward1, true);

        });
        setPathToSpit1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toSpit1, true);
        });
        setPathToSpit2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toSpit2, true);
        });
        setPathToToSpecimen2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toSpecimen2, true);
        });
        setPathToScore2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore2, true);
            autoDriveSubsystem.setMaxPower(1);
        });
        setPathToForward2 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(forward2, true);
            autoDriveSubsystem.setMaxPower(1);
        });
        setPathToToSpecimen3 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toSpecimen3, true);
        });

        setPathToScore3 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore3, true);
            autoDriveSubsystem.setMaxPower(1);
        });

        PushSample = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(Backup, true);
            autoDriveSubsystem.setMaxPower(1);
        });

        setPathToPark = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPark, false);
        });



        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new WaitCommand(50),
                new RaiseWrist(wrist),
                new RetractCommand(extend),
                new SwingArmDownCommand(swingArmSubsystem)
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
                        new AutoClipSpecimen(liftSubsystem, 200),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry), new RetractCommand(extend),
                        new LiftBottomCommand(liftSubsystem)

                ),
                setPathToGrabSpecimen,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                new LiftTopBarCommand(liftSubsystem),
                setPathToScore1,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                setPathToForward1,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                setPathToBackAwayFromBar,
                new ParallelCommandGroup(
                        new AutoClipSpecimen(liftSubsystem, 200),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToToSpecimen2,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftBottomCommand(liftSubsystem)
                ),
                setPathToGrabSpecimen,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new SequentialCommandGroup(
                                new SwingArmDownCommand(swingArmSubsystem),
                                new WaitCommand(200),
                                new LiftForSwingArmClearCommand(liftSubsystem)
                        )
                ),
                setPathToScore2,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftTopBarCommand(liftSubsystem)
                ),
                setPathToForward2,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                setPathToBackAwayFromBar,
                new ParallelCommandGroup(
                        new AutoClipSpecimen(liftSubsystem, 200),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToToSpecimen3,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftBottomCommand(liftSubsystem)
                ),
                setPathToGrabSpecimen,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new SequentialCommandGroup(
                                new SwingArmDownCommand(swingArmSubsystem),
                                new WaitCommand(200),
                                new LiftForSwingArmClearCommand(liftSubsystem)
                        )
                ),
                setPathToScore3,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftTopBarCommand(liftSubsystem)
                ),
                new LiftBottomCommand(liftSubsystem)
        );


        schedule(new SequentialCommandGroup(
                number5IsAlive,
                new InstantCommand(() -> {
                    follower.breakFollowing();
                    requestOpModeStop();
                })
        ));
    }

    @Override
    public void makeAuto() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(Start);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extend = new ExtendSubsystem(touch2, hardwareMap.get(DcMotorEx.class, "extensionMotor"));
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
        toBar.setPathEndTimeoutConstraint(25);

        toScorePreload = new Path(new BezierCurve(new Point(BarMid), new Point(Bar)));
        toScorePreload.setLinearHeadingInterpolation(BarMid.getHeading(), Bar.getHeading());
        toScorePreload.setPathEndTimeoutConstraint(500);

        backAwayFromBar = new Path(new BezierCurve(new Point(Bar), new Point(BackAwayFromBar)));
        backAwayFromBar.setLinearHeadingInterpolation(Bar.getHeading(), BackAwayFromBar.getHeading());
        backAwayFromBar.setPathEndTimeoutConstraint(25);

        Path strafe1 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(StrafeOver)));
        strafe1.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), StrafeOver.getHeading());

        Path clear1 = new Path(new BezierCurve(new Point(StrafeOver), new Point(Clear1)));
        clear1.setLinearHeadingInterpolation(StrafeOver.getHeading(), Clear1.getHeading());

        Path over1 = new Path(new BezierCurve(new Point(Clear1), new Point(Over1)));
        over1.setLinearHeadingInterpolation(Clear1.getHeading(), Over1.getHeading());


        Path push1 = new Path(new BezierCurve(new Point(Over1), new Point(Push1)));
        push1.setLinearHeadingInterpolation(Over1.getHeading(), Push1.getHeading());

        Path to2 = new Path(new BezierCurve(new Point(Push1), new Point(Over1)));
        to2.setLinearHeadingInterpolation(Push1.getHeading(), Over1.getHeading());

        Path over2 = new Path(new BezierCurve(new Point(Over1), new Point(Over2)));
        over2.setLinearHeadingInterpolation(Over1.getHeading(), Over2.getHeading());

        Path push2 = new Path(new BezierCurve(new Point(Over2), new Point(Push2)));
        push2.setLinearHeadingInterpolation(Over2.getHeading(), Push2.getHeading());

        Path strafe2 = new Path(new BezierCurve(new Point(Push2), new Point(StrafeOver2)));
        strafe2.setLinearHeadingInterpolation(Push2.getHeading(), StrafeOver2.getHeading());



        run1 = new PathChain(strafe1, clear1, over1, push1, to2, over2, push2, strafe2);

        toSpecimen1 = new Path(new BezierCurve(new Point(StrafeOver2), new Point(ForSpecimen)));
        toSpecimen1.setLinearHeadingInterpolation(StrafeOver2.getHeading(), ForSpecimen.getHeading());
        toSpecimen1.setPathEndTimeoutConstraint(500);


        toGrabSpecimen = new Path(new BezierCurve(new Point(ForSpecimen), new Point(GrabSpecimen)));
        toGrabSpecimen.setLinearHeadingInterpolation(ForSpecimen.getHeading(), GrabSpecimen.getHeading());
        toGrabSpecimen.setPathEndTimeoutConstraint(500);

        toScore1 = new Path(new BezierCurve(new Point(GrabSpecimen), new Point(new Pose(BarMid.getX()+3, BarMid.getY()-2, BarMid.getHeading()))));
        toScore1.setLinearHeadingInterpolation(GrabSpecimen.getHeading(), BarMid.getHeading());
        toScore1.setPathEndTimeoutConstraint(500);

        forward1 = new Path(new BezierCurve(new Point(BarMid), new Point(new Pose(Bar.getX()+2, Bar.getY(), Bar.getHeading()))));
        forward1.setLinearHeadingInterpolation(BarMid.getHeading(), new Pose(Bar.getX()+2, Bar.getY(), Bar.getHeading()).getHeading());
        forward1.setPathEndTimeoutConstraint(500);

        toSpecimen2 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(ForSpecimen)));
        toSpecimen2.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), ForSpecimen.getHeading());
        toSpecimen2.setPathEndTimeoutConstraint(500);

        toScore2 = new Path(new BezierCurve(new Point(GrabSpecimen), new Point(new Pose(BarMid.getX()-4, BarMid.getY()-2.5, BarMid.getHeading()))));
        toScore2.setLinearHeadingInterpolation(GrabSpecimen.getHeading(), BarMid.getHeading());
        toScore2.setPathEndTimeoutConstraint(250);

        forward2 = new Path(new BezierCurve(new Point(new Pose(BarMid.getX()-4, BarMid.getY()-2.5, BarMid.getHeading())), new Point(new Pose(Bar.getX()-4, Bar.getY(), Bar.getHeading()))));
        forward2.setLinearHeadingInterpolation(BarMid.getHeading(), new Pose(Bar.getX()-4, Bar.getY(), Bar.getHeading()).getHeading());
        forward2.setPathEndTimeoutConstraint(500);

        toSpecimen3 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(ForSpecimen)));
        toSpecimen3.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), ForSpecimen.getHeading());
        toSpecimen3.setPathEndTimeoutConstraint(500);

        toScore3 = new Path(new BezierCurve(new Point(GrabSpecimen), new Point(new Pose(Bar.getX()-7, Bar.getY(), Bar.getHeading()))));
        toScore3.setLinearHeadingInterpolation(GrabSpecimen.getHeading(), Bar.getHeading());
        toScore3.setPathEndTimeoutConstraint(500);



    }


}


