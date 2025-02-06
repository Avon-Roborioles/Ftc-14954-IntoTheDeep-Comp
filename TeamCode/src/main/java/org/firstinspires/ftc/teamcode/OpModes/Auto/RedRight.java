package org.firstinspires.ftc.teamcode.OpModes.Auto;



import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.HalfChassisLength;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.HalfChassisWidth;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLStartBar;
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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntakeForEject;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.IntakeToReadyForEject;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LeverCommands.LeverClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoClipSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.ClipTopSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassEject;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
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
public class RedRight extends AutoBase{
    public Pose Start = new Pose( HalfChassisWidth -0.75 ,-68.5 + HalfChassisLength, -PI/2);
    public Pose Bar = new Pose( HalfChassisWidth -0.75 , -31.5, -PI/2);
    public Pose BarMid = new Pose( HalfChassisWidth -0.75 , -38, -PI/2);
    public Pose BackAwayFromBar = new Pose( HalfChassisWidth -0.75 , -34, -PI/2);
    public Pose Grab1 = new Pose(44, -48, PI/2);
    public Pose Grab2 = new Pose(54.5, -48, PI/2);
    public Pose ForSpecimen = new Pose(30.5, -58, PI/2);
    public Pose GrabSpecimen = new Pose(ForSpecimen.getX(),Start.getY()-0.5, PI/2);



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
            autoDriveSubsystem.setMaxPower(0.75);
        });
        setPathToScore1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toScore1, true);
        });
        setPathToForward1 = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(forward1, true);
        });







        setPathToPark = new InstantCommand(() -> {
            autoDriveSubsystem.followPath(toPark, false);
        });



        SequentialCommandGroup initSubsystems = new SequentialCommandGroup(
                new LeverClearCommand(lever),
                new WaitCommand(100),
                new HandoffCommand(wrist),
                new RetractCommand(extend),
                new SwingArmDownCommand(swingArmSubsystem)
        );
        ParallelCommandGroup spitAndReady= new ParallelCommandGroup(
                new PassEject(pass),
                new ExtensionCommand(extend, 0.64),
                new AutoDriveCommand(autoDriveSubsystem, telemetry)
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
                        new AutoClipSpecimen(liftSubsystem, 500),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
                ),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new ExtensionCommand(extend, 0.64),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
                ),
                new AutoIntakeForEject(intake, wrist, pass, extend, liftSubsystem),
                setPathToPickUp2,
                spitAndReady,
                new AutoIntakeForEject(intake, wrist, pass, extend, liftSubsystem),
                setPathToToSpecimen1,
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new PassEject(pass),
                                new LiftBottomCommand(liftSubsystem)
                        ),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)
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
                        new AutoClipSpecimen(liftSubsystem, 500),
                        new SequentialCommandGroup(
                                new WaitCommand(250),
                                new AutoDriveCommand(autoDriveSubsystem, telemetry)
                        )
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
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(RLStartBar);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(Servo.class, "extension");
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
        toPickUp1.setPathEndTimeoutConstraint(1500);

        toPickUp2 = new Path(new BezierCurve(new Point(Grab1), new Point(Grab2)));
        toPickUp2.setLinearHeadingInterpolation(Grab1.getHeading(), Grab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(1000);

        toSpecimen1 = new Path(new BezierCurve(new Point(Grab2), new Point(ForSpecimen)));
        toSpecimen1.setLinearHeadingInterpolation(Grab2.getHeading(), ForSpecimen.getHeading());
        toSpecimen1.setPathEndTimeoutConstraint(1000);

        toGrabSpecimen = new Path(new BezierCurve(new Point(ForSpecimen), new Point(GrabSpecimen)));
        toGrabSpecimen.setLinearHeadingInterpolation(ForSpecimen.getHeading(), GrabSpecimen.getHeading());
        toGrabSpecimen.setPathEndTimeoutConstraint(500);

        toScore1 = new Path(new BezierCurve(new Point(GrabSpecimen), new Point(new Pose(BarMid.getX()+2, BarMid.getY(), BarMid.getHeading()))));
        toScore1.setLinearHeadingInterpolation(GrabSpecimen.getHeading(), BarMid.getHeading());
        toScore1.setPathEndTimeoutConstraint(750);

        forward1 = new Path(new BezierCurve(new Point(BarMid), new Point(new Pose(Bar.getX()+2, Bar.getY(), Bar.getHeading()))));
        forward1.setLinearHeadingInterpolation(BarMid.getHeading(), new Pose(Bar.getX()+2, Bar.getY(), Bar.getHeading()).getHeading());
        forward1.setPathEndTimeoutConstraint(500);








    }


}
