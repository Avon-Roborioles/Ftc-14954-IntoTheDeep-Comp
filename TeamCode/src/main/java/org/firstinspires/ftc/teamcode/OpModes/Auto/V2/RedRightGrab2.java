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
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.OpModes.Auto.AutoBase;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoCollectNoColorSample;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntakeForEject;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.Reject;
import org.firstinspires.ftc.teamcode.commands.LeverCommands.LeverClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLastClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassEject;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
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
public class RedRightGrab2 extends AutoBase {
    public Pose Start = new Pose( 7.8125 ,-60.0125, -PI/2);
    public Pose Bar = new Pose( 7.8125 , -31.5, -PI/2);
    public Pose BarMid = new Pose( 7.8125 , -38, -PI/2);
    public Pose BackAwayFromBar = new Pose( 8.5625-0.75 , -34, -PI/2);
    public Pose Grab1 = new Pose(25, -41, PI/4);
    public Pose Spit1 = new Pose(40, -38, -PI/4);
    public Pose Grab2 = new Pose(38, -41, PI/4);

    public Pose ForSpecimen = new Pose(30.5, -56, PI/2);
    public Pose GrabSpecimen = new Pose(ForSpecimen.getX(),Start.getY()-1, PI/2);



    public Pose Park = new Pose(60, -60, PI);
    public Pose ParkMid = new Pose(40, -40, PI);

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

        setPathToPark = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPark, false);
        });



        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new LeverClearCommand(lever),
                new WaitCommand(50),
                new HandoffCommand(wrist),
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
                        new ExtensionCommand(extend, 0.64),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new ExtensionCommand(extend, 0.64),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new LiftBottomCommand(liftSubsystem)
                ),
                new LowerWrist(wrist),
                new ParallelCommandGroup(new AutoCollectNoColorSample(intake, wrist),
                        new ExtensionCommand(extend,0.5)),
                setPathToSpit1,
                new ParallelCommandGroup(
                        new HandoffCommand(wrist),
                        new ExtensionCommand(extend, 0.64),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
                ),
                setPathToPickUp2,
                new Reject(intake),
                new ParallelCommandGroup(
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new ExtensionCommand(extend, 0.64)
                ),
                new ParallelCommandGroup(new LowerWrist(wrist), new AutoCollectNoColorSample(intake, wrist),
                        new ExtensionCommand(extend,0.5)),
                setPathToSpit2,
                new ParallelCommandGroup(
                        new HandoffCommand(wrist),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry),
                        new ExtensionCommand(extend, 0.7)
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
        extendservo = hardwareMap.get(ServoImplEx.class, "extension");
        extendservo.setPwmRange(servoRange);
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"), hardwareMap.get(Rev2mDistanceSensor.class, "passDistance"), intake);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class,"boxDistance"),intake);
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
        toPickUp1.setPathEndTimeoutConstraint(750);

        toSpit1 = new Path(new BezierCurve(new Point(Grab1), new Point(Spit1)));
        toSpit1.setLinearHeadingInterpolation(Grab1.getHeading(), Spit1.getHeading());
        toSpit1.setPathEndTimeoutConstraint(250);


        toPickUp2 = new Path(new BezierCurve(new Point(Spit1), new Point(Grab2)));
        toPickUp2.setLinearHeadingInterpolation(Spit1.getHeading(), Grab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(750);

        toSpit2 = new Path(new BezierCurve(new Point(Grab2), new Point(Spit1)));
        toSpit2.setLinearHeadingInterpolation(Grab2.getHeading(), Spit1.getHeading());
        toSpit2.setPathEndTimeoutConstraint(250);


        toSpecimen1 = new Path(new BezierCurve(new Point(Grab2), new Point(ForSpecimen)));
        toSpecimen1.setLinearHeadingInterpolation(Grab2.getHeading(), ForSpecimen.getHeading());
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
    }


}
