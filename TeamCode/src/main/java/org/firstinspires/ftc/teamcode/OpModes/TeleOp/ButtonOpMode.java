package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;


import org.firstinspires.ftc.teamcode.commands.CommandGroups.BottomBucketScoreReady;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.IntakeToReadyForTopScore;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.TopBucketScoreReady;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CancelCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.Reject;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomResetCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmScoreCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;

import org.firstinspires.ftc.teamcode.commands.TelemetryCommand;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name = "ButtonOpMode", group = "Test Op Modes")
public class ButtonOpMode extends CommandOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight, liftMotor;
    private GamepadEx driverOp, operatorOp;
    private TouchSensor touch1, touch2;
    private ServoImplEx extendservo;
    PwmControl.PwmRange servoRange = new PwmControl.PwmRange(799, 1500);

    private PedroDriveSubsystem pedroDriveSubsystem;
    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private PassSubsystem pass;
    private IntakeSubsystem intake;
    private BoxxySubsystem box;
    private WristSubsystem wrist;
    private TelemetrySubsystem telemetrySubsystem;



    @Override
    public void initialize() {

        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "backLeft", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "backRight", Motor.GoBILDA.RPM_312);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        extendservo = hardwareMap.get(ServoImplEx.class, "extension");
        extendservo.setPwmRange(servoRange);
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor, touch1);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"), hardwareMap.get(Rev2mDistanceSensor.class, "passDistance"),intake);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        box = new BoxxySubsystem(hardwareMap.get(DistanceSensor.class, "boxDistance"), intake);

        intake = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor1"),hardwareMap.get(ColorSensor.class, "intakeColor2"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(DistanceSensor.class, "intakeDistance"), hardwareMap.get(ServoImplEx.class, "allianceColor"), true, hardwareMap.get(CRServo.class, "intakeRoller"));
        telemetrySubsystem = new TelemetrySubsystem(telemetry, box, extend, intake, liftSubsystem, pass, pedroDriveSubsystem, swingArmSubsystem, wrist);
        //Default Commands
        telemetrySubsystem.setDefaultCommand(new TelemetryCommand(telemetrySubsystem));
        pass.setDefaultCommand(new PassCommand(pass, operatorOp::getLeftY));


        /*
        Command Bindings
         */
        operatorOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new LiftTopCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftTopBarCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new LiftForSwingArmClearCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new LiftBottomCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new LiftBottomResetCommand(liftSubsystem));

        operatorOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new HandoffCommand(wrist))
                .toggleWhenPressed(new ExtendCommand(extend), new RetractCommand(extend));

        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new Reject(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new CollectSample(intake, wrist));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ToggleAlliance(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new Reject(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new CancelCommand(intake, pass, liftSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new EjectCommand(intake, pass));

        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new SwingArmScoreCommand(swingArmSubsystem, box, intake), new SwingArmDownCommand(swingArmSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new SwingArmMidCommand(swingArmSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new HandoffCommand(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LowerWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new RaiseWrist(wrist));


//                .whenPressed(new PassToBox(pass, box, intake));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new SwingArmDownCommand(swingArmSubsystem));
//                .toggleWhenPressed(new PassOnToBoxCommand(pass, box, intake), new PassOffCommand(pass));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new TopBucketScoreReady(swingArmSubsystem, liftSubsystem, pass));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new BottomBucketScoreReady(swingArmSubsystem, liftSubsystem, pass));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new Score(swingArmSubsystem, liftSubsystem, box, intake));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new IntakeToReadyForTopScore(intake, wrist, pass, extend, swingArmSubsystem, box, liftSubsystem));
        CommandScheduler.getInstance().schedule(new ExtendCommand(extend), new RetractCommand(extend));

//        Sample Trigger code
//        Trigger intakeTrigger = new Trigger(()-> driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>.7);
//        intakeTrigger.whenActive();
        /*
        *        Open:
        *
        *      -Buttons:
        *    -Operator Op:
        *X
        *
        *    -Driver Op:
        *Right Bumper
        *Right Stick Button
        *Left Stick Button
        *Start
        *
        *      -Analog:
        *
        *    -Operator Op:
        *Right Stick X
        *Right Stick Y
        *Left Stick X
        *Right Trigger
        *Left Trigger
        *
        *    -Driver Op:
        *Right Stick Y
        *Right Trigger
        *Left Trigger
         */
    }
}
