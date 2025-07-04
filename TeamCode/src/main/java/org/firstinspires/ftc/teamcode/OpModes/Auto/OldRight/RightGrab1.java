package org.firstinspires.ftc.teamcode.OpModes.Auto.OldRight;

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
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.OpModes.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoCollectNoColorSample;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.Reject;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLastClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
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

@Autonomous
@Disabled
public class RightGrab1 extends AutoBase {
    public Pose Start = new Pose( 7.8125 ,-60.0125, -PI/2);
    public Pose Bar = new Pose( 7.8125 , -31.5, -PI/2);
    public Pose BarMid = new Pose( 7.8125 , -38, -PI/2);
    public Pose BackAwayFromBar = new Pose( 8.5625-0.75 , -34, -PI/2);
    public Pose Grab1 = new Pose(25, -41, PI/4);
    public Pose Spit1 = new Pose(40, -38, -PI/4);
    public Pose Grab2 = new Pose(38, -41, PI/4);

    public Pose ForSpecimen = new Pose(30.5, -56, PI/2);
    public Pose GrabSpecimen = new Pose(ForSpecimen.getX(),Start.getY()-1.25, PI/2);



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
            autoDriveSubsystem.followPath(toPickUp1, true);
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
                        new AutoClipSpecimen(liftSubsystem, 250),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new ExtensionCommand(extend, 0.6),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftBottomCommand(liftSubsystem)
                ),
                new LowerWrist(wrist),
                new ParallelCommandGroup(new AutoCollectNoColorSample(intake, wrist, 2000),
                        new ExtensionCommand(extend,0.5)),
                setPathToSpit1,
                new SequentialCommandGroup(
                        new RaiseWrist(wrist),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
                ),
                new Reject(intake),
                setPathToToSpecimen1,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new RetractCommand(extend)
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
                        new AutoClipSpecimen(liftSubsystem, 250),
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
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                new LiftTopBarCommand(liftSubsystem),
                setPathToScore2,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                setPathToForward2,
                new AutoDriveCommand(autoDriveSubsystem, telemetry),
                setPathToBackAwayFromBar,
                new ParallelCommandGroup(
                        new AutoLastClipSpecimen(liftSubsystem, 250),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToPark,
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftBottomCommand(liftSubsystem)
                )


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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(Start);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extend = new ExtendSubsystem(touch2, hardwareMap.get(DcMotorEx.class, "extensionMotor"));
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        claw = new ClawSubsystem(hardwareMap.get(Servo.class, "claw"));
        intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"));
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
        toScorePreload.setPathEndTimeoutConstraint(750);

        backAwayFromBar = new Path(new BezierCurve(new Point(Bar), new Point(BackAwayFromBar)));
        backAwayFromBar.setLinearHeadingInterpolation(Bar.getHeading(), BackAwayFromBar.getHeading());
        backAwayFromBar.setPathEndTimeoutConstraint(250);

        toPickUp1 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(Grab1)));
        toPickUp1.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), Grab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(500);

        toSpit1 = new Path(new BezierCurve(new Point(Grab1), new Point(Spit1)));
        toSpit1.setLinearHeadingInterpolation(Grab1.getHeading(), Spit1.getHeading());
        toSpit1.setPathEndTimeoutConstraint(250);


        toSpecimen1 = new Path(new BezierCurve(new Point(Spit1), new Point(ForSpecimen)));
        toSpecimen1.setLinearHeadingInterpolation(Spit1.getHeading(), ForSpecimen.getHeading());
        toSpecimen1.setPathEndTimeoutConstraint(1000);

        toGrabSpecimen = new Path(new BezierCurve(new Point(ForSpecimen), new Point(GrabSpecimen)));
        toGrabSpecimen.setLinearHeadingInterpolation(ForSpecimen.getHeading(), GrabSpecimen.getHeading());
        toGrabSpecimen.setPathEndTimeoutConstraint(500);

        toScore1 = new Path(new BezierCurve(new Point(GrabSpecimen), new Point(new Pose(BarMid.getX()+2, BarMid.getY(), BarMid.getHeading()))));
        toScore1.setLinearHeadingInterpolation(GrabSpecimen.getHeading(), BarMid.getHeading());
        toScore1.setPathEndTimeoutConstraint(500);

        forward1 = new Path(new BezierCurve(new Point(BarMid), new Point(new Pose(Bar.getX()+2, Bar.getY(), Bar.getHeading()))));
        forward1.setLinearHeadingInterpolation(BarMid.getHeading(), new Pose(Bar.getX()+2, Bar.getY(), Bar.getHeading()).getHeading());
        forward1.setPathEndTimeoutConstraint(500);

        toSpecimen2 = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(ForSpecimen)));
        toSpecimen2.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), ForSpecimen.getHeading());
        toSpecimen2.setPathEndTimeoutConstraint(500);

        toScore2 = new Path(new BezierCurve(new Point(GrabSpecimen), new Point(new Pose(BarMid.getX()-2, BarMid.getY(), BarMid.getHeading()))));
        toScore2.setLinearHeadingInterpolation(GrabSpecimen.getHeading(), BarMid.getHeading());
        toScore2.setPathEndTimeoutConstraint(500);

        forward2 = new Path(new BezierCurve(new Point(new Pose(BarMid.getX()-2, BarMid.getY(), BarMid.getHeading())), new Point(new Pose(Bar.getX()-2, Bar.getY(), Bar.getHeading()))));
        forward2.setLinearHeadingInterpolation(BarMid.getHeading(), new Pose(Bar.getX()-2, Bar.getY(), Bar.getHeading()).getHeading());
        forward2.setPathEndTimeoutConstraint(500);

        toPark = new Path(new BezierCurve(new Point(BackAwayFromBar), new Point(Park)));
        toPark.setLinearHeadingInterpolation(BackAwayFromBar.getHeading(), Park.getHeading());
        toPark.setPathEndTimeoutConstraint(500);

    }
}