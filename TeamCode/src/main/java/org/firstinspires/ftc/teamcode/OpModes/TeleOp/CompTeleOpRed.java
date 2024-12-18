package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CancelCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.IntakeScore;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.TelemetryCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name = "CompTeleOpRed")
public class CompTeleOpRed extends CommandOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight, liftMotor;
    private GamepadEx driverOp, operatorOp;
    private TouchSensor touch1, touch2;
    private Servo extendservo;

    private Follower follower;
    private PedroDriveSubsystem pedroDriveSubsystem;
    private DriveSubsystem drive;

//    private Telemetry telemetry;

    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private PassSubsystem pass;
    private IntakeSubsystem intake;
    private BoxxySubsystem box;
    private WristSubsystem wrist;
    private TelemetrySubsystem telemetrySubsystem;

    /*   Ideas to add
    *Once the box Distance goes back to normal as the sample leaves put the swing arm and lift down.
    *
     */

    @Override
    public void initialize() {
//        telemetry =   new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry = new Telemetry(telemetry);
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        frontLeft = new Motor(hardwareMap,"frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap,"frontRight", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap,"backLeft", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap,"backRight", Motor.GoBILDA.RPM_312);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        extendservo = hardwareMap.get(Servo.class, "extension");
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"));
        liftSubsystem = new LiftSubsystem(liftMotor,touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"));
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class,"boxDistance"));



//        follower = new Follower(hardwareMap);
//        follower.startTeleopDrive();
//        follower.setMaxPower(1);
//
//
//        pedroDriveSubsystem = new PedroDriveSubsystem( follower);
//        pedroDriveSubsystem.setDefaultCommand(new PedroDriveCommand(pedroDriveSubsystem, telemetry, driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX, true));


        drive = new DriveSubsystem(frontLeft, frontRight, backLeft, backRight);


        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"));

//        telemetrySubsystem = new TelemetrySubsystem(telemetry, box, extend, intake, liftSubsystem, pass, pedroDriveSubsystem, swingArmSubsystem, wrist);

        //Default Commands
        drive.setDefaultCommand(new DriveCommand(drive, driverOp::getLeftX,driverOp::getLeftY,driverOp::getRightX));
//        telemetrySubsystem.setDefaultCommand(new TelemetryCommand(telemetrySubsystem));


        /*
        Command Bindings
         */


        //Swing Arm (X & Y)
        operatorOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SwingArmUpCommand(swingArmSubsystem, box));
        operatorOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SwingArmDownCommand(swingArmSubsystem));




        //Extend (Left Bumper)
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new HandoffCommand(wrist))
                .toggleWhenPressed(new ExtendCommand(extend), new RetractCommand(extend));

        //Intake (D-Pad &  driver op D Down is alliance)
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new Score(swingArmSubsystem, liftSubsystem,box));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new CancelCommand(intake, pass));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new IntakeScore(intake, wrist, pass,extend,swingArmSubsystem,box,liftSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ToggleAlliance(intake));

        //Lift ( B & Stick Buttons)
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new LiftTopBarCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftBottomCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new LiftTopCommand(liftSubsystem));

        /* Open Buttons
        * right bumper
        * Start
        * Back
        * Dpad Left
        * Right Stick Button
        *
        *   Open Triggers (Different Methods)
        * Triggers
        * Joysticks
         */
    }
}
