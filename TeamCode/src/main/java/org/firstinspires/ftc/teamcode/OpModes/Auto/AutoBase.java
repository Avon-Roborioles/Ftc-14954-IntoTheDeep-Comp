package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;

public abstract class AutoBase extends CommandOpMode {
//    protected Follower follower;
//
//    protected Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//    protected Limelight3A limelight3A;
//    protected AutoDriveSubsystem autoDriveSubsystem;
//    protected AutoDriveCommand autoDriveCommand;
//    protected CameraAdjustCommand cameraAdjustCommand;
//    protected LimelightSubsystem limelightSubsystem;
//
//    protected TouchSensor touch1, touch2;
//    protected Servo extendservo;
//    protected Motor liftMotor;
//    protected ExtendSubsystem extend;
//    protected LiftSubsystem liftSubsystem;
//    protected SwingArmSubsystem swingArmSubsystem;
//    protected PassSubsystem pass;
//    protected IntakeSubsystem intake;
//    protected BoxxySubsystem box;
//    protected WristSubsystem wrist;
//
//    Path toScan, toScorePreload, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark;
//    Command setPathToScan, setPathToScorePreload, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, camera;


    public void subInit(boolean RedAlliance) {

    }
    public abstract void makeAuto();
    public abstract void buildPaths();



}
