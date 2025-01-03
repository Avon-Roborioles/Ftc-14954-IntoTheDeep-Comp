package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutonomusCommands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.commands.VisionCommands.CameraAdjustCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public abstract class AutoBase extends CommandOpMode {
    protected Follower follower;

    protected Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    protected Limelight3A limelight3A;
    protected AutoDriveSubsystem autoDriveSubsystem;
    protected AutoDriveCommand autoDriveCommand;
    protected CameraAdjustCommand cameraAdjustCommand;
    protected LimelightSubsystem limelightSubsystem;

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
    protected TelemetrySubsystem telemetrySubsystem;

    public void subInit(boolean RedAlliance) {
        follower = new Follower(hardwareMap);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        extendservo = hardwareMap.get(Servo.class, "extension");
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"));
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class,"boxDistance"));
        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), RedAlliance);

    }
    public abstract void makeAuto();

    @Override
    public void initialize() {
        makeAuto();
    }
}
