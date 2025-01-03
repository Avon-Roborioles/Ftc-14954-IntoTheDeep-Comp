package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLGrab1;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLGrab2;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLGrab3;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLPark;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLScan;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLScore;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.PoseList.BLStart;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.VisionCommands.CameraAdjustCommand;
import org.firstinspires.ftc.teamcode.oldPedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.oldPedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.oldPedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;

@Autonomous
public class BlueLeft extends AutoBase{
    @Override
    public void buildPaths() {
        toScan = new Path(new BezierCurve(new Point(BLStart), new Point(BLScan)));
        toScan.setLinearHeadingInterpolation(BLStart.getHeading(), BLScan.getHeading());
        toScan.setPathEndTimeoutConstraint(1500);

        toScorePreload = new Path(new BezierCurve(new Point(BLScan), new Point(BLScore)));
        toScorePreload.setLinearHeadingInterpolation(BLScan.getHeading(), BLScore.getHeading());
        toScorePreload.setPathEndTimeoutConstraint(1);

        toPickUp1 = new Path(new BezierCurve(new Point(BLScore), new Point(BLGrab1)));
        toPickUp1.setLinearHeadingInterpolation(BLScore.getHeading(), BLGrab1.getHeading());
        toPickUp1.setPathEndTimeoutConstraint(1);

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
    }

    @Override
    public void buildCommands() {
        setPathToScan = new InstantCommand(() -> {
            autoDriveSubsystem.setMaxPower(0.01);
            autoDriveSubsystem.followPath(toScan, true);
        });
        setPathToScorePreload = new InstantCommand(() -> {
            autoDriveSubsystem.setMaxPower(1);
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
        camera = new ParallelCommandGroup(new WaitCommand(1000), cameraAdjustCommand);
    }


    @Override
    public void makeAuto() {
        subInit(false);
        buildPaths();

        follower.setStartingPose(BLStart);
        autoDriveSubsystem = new AutoDriveSubsystem(follower, mTelemetry, BLStart);
        autoDriveSubsystem.setMaxPower(0.01);
        autoDriveCommand = new AutoDriveCommand(autoDriveSubsystem, mTelemetry);
        cameraAdjustCommand = new CameraAdjustCommand(autoDriveSubsystem, limelightSubsystem, mTelemetry);
        register(extend, liftSubsystem, swingArmSubsystem, pass, intake, box, wrist, autoDriveSubsystem, limelightSubsystem);
        buildCommands();

        schedule(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
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
                    ),
                    new WaitCommand(29500)
                ),
                new InstantCommand(() -> {
                    follower.breakFollowing();
                    reset();
                })
            )
        );







    }

}
