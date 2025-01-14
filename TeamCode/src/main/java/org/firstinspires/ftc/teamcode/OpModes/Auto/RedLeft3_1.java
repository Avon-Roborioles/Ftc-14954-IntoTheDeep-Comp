package org.firstinspires.ftc.teamcode.OpModes.Auto;



import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLBar;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLBarMid;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLForward1;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLForward2;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLForward3;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLGrab1;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLGrab1Mid;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLGrab2;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLGrab3;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLGrab3Mid;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLPark;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLParkMid;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLScore;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.RLStartBar;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoAfterScore;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoEndCommand;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoToScore;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
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
public class RedLeft3_1 extends AutoBase{


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
                new WaitCommand(10),
                new HandoffCommand(wrist),
                new RetractCommand(extend),
                new SwingArmDownCommand(swingArmSubsystem)
        );
        ParallelCommandGroup IntakeAndDrive =  new ParallelCommandGroup(
                new AutoIntake(intake, wrist),
                new LiftBottomCommand(liftSubsystem),
                new AutoDriveCommand(autoDriveSubsystem, telemetry));

        SequentialCommandGroup number5IsAlive = new SequentialCommandGroup(
                initSubsystems,
                setPathToBar,
                new ParallelCommandGroup(
                        new LiftTopBarCommand(liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                setPathToScorePreload,
                new ParallelCommandGroup(
                        new LiftTopBarCommand(liftSubsystem),
                        new AutoDriveCommand(autoDriveSubsystem, telemetry)),
                new LiftForSwingArmClearCommand(liftSubsystem),
                setPathToPickUp1,
                new ParallelCommandGroup(
                        new LiftBottomCommand(liftSubsystem),
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
        follower.setStartingPose(RLStartBar);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(Servo.class, "extension");
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"), hardwareMap.get(Rev2mDistanceSensor.class, "passDistance"));
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class,"boxDistance"));
        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor1"),hardwareMap.get(ColorSensor.class, "intakeColor2"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), true);
        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, RLStartBar);
    }

    @Override
    public void buildPaths() {
        toBar = new Path(new BezierCurve(new Point(RLStartBar), new Point(RLBarMid)));
        toBar.setLinearHeadingInterpolation(RLStartBar.getHeading(), RLBarMid.getHeading());
        toBar.setPathEndTimeoutConstraint(250);

        toScorePreload = new Path(new BezierCurve(new Point(RLBarMid), new Point(RLBar)));
        toScorePreload.setLinearHeadingInterpolation(RLBarMid.getHeading(), RLBar.getHeading());


        toPickUp1 = new Path(new BezierCurve(new Point(RLScore),new Point(RLGrab1Mid), new Point(RLGrab1)));
        toPickUp1.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(500);

        toScore1 = new Path(new BezierCurve(new Point(RLGrab1), new Point(RLScore)));
        toScore1.setLinearHeadingInterpolation(RLGrab1.getHeading(), RLScore.getHeading());
        toScore1.setPathEndTimeoutConstraint(2000);

        toPickUp2 = new Path(new BezierCurve(new Point(RLScore), new Point(RLGrab2)));
        toPickUp2.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab2.getHeading());
        toPickUp2.setPathEndTimeoutConstraint(1000);

        toScore2 = new Path(new BezierCurve(new Point(RLGrab2), new Point(RLScore)));
        toScore2.setLinearHeadingInterpolation(RLGrab2.getHeading(), RLScore.getHeading());
        toScore2.setPathEndTimeoutConstraint(1000);

        toPickUp3 = new Path(new BezierCurve(new Point(RLScore),new Point(RLGrab3Mid), new Point(RLGrab3)));
        toPickUp3.setLinearHeadingInterpolation(RLScore.getHeading(), RLGrab3.getHeading());
        toPickUp3.setPathEndTimeoutConstraint(1000);

        toScore3 = new Path(new BezierCurve(new Point(RLGrab3), new Point(RLScore)));
        toScore3.setLinearHeadingInterpolation(RLGrab3.getHeading(), RLScore.getHeading());
        toScore3.setPathEndTimeoutConstraint(1000);

        toPark = new Path(new BezierCurve(new Point(RLScore),new Point(RLParkMid), new Point(RLPark)));
        toPark.setLinearHeadingInterpolation(RLScore.getHeading(), RLPark.getHeading());
        toPark.setPathEndTimeoutConstraint(1000);

        forward1 = new Path(new BezierCurve(new Point(RLGrab1), new Point(RLForward1)));
        forward1.setConstantHeadingInterpolation(RLGrab1.getHeading());

        forward2 = new Path(new BezierCurve(new Point(RLGrab2), new Point(RLForward2)));
        forward2.setConstantHeadingInterpolation(RLGrab2.getHeading());

        forward3 = new Path(new BezierCurve(new Point(RLGrab3), new Point(RLForward3)));
        forward3.setConstantHeadingInterpolation(RLGrab3.getHeading());
    }


}
