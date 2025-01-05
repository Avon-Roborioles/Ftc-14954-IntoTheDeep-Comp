package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public abstract class AutoBase extends CommandOpMode {
    protected Follower follower;
    protected Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    protected AutoDriveSubsystem autoDriveSubsystem;
    protected TouchSensor touch1, touch2;
    protected Servo extendservo;
    protected Motor liftMotor;
    protected ExtendSubsystem extend;
    protected LiftSubsystem liftSubsystem;
    protected SwingArmSubsystem swingArmSubsystem;
    protected PassSubsystem pass;
    protected IntakeSubsystem intake;
    protected BoxxySubsystem box;
    protected WristSubsystem wrist;
    protected Command setPathToScorePreload, setPathToPickUp1, setPathToScore1, setPathToPickUp2, setPathToScore2, setPathToPickUp3, setPathToScore3, setPathToPark, setPathToForward1, setPathToForward2, setPathToForward3;
    protected Path toScorePreload, toPickUp1,toScore1, toPickUp2, toScore2, toPickUp3, toScore3, toPark, forward1, forward2, forward3;

    public abstract void makeAuto();
    public abstract void buildPaths();



}
