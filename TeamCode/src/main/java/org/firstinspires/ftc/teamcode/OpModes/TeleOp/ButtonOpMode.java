package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.teamcode.commands.CommandGroups.IntakeToReadyForTopScore;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ZeroExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.Reject;
//import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.SpitCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomResetCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftForSwingArmClearCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;

import org.firstinspires.ftc.teamcode.commands.TelemetryCommand;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name = "ButtonOpMode", group = "Test Op Modes")
public class ButtonOpMode extends CommandOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight, liftMotor;
    private GamepadEx driverOp, operatorOp;
    private TouchSensor touch1, touch2;

    private PedroDriveSubsystem pedroDriveSubsystem;
    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private NewIntakeSubsystem intake;
    private WristSubsystem wrist;
    private ClawSubsystem claw;
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

        extend = new ExtendSubsystem(touch2, hardwareMap.get(DcMotorEx.class, "extensionMotor"));
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor, touch1);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"),hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"), true);
        telemetrySubsystem = new TelemetrySubsystem(telemetry, extend,  liftSubsystem, pedroDriveSubsystem, swingArmSubsystem, wrist);
        claw = new ClawSubsystem(hardwareMap.get(ServoImplEx.class, "claw"));
        //Default Commands
        telemetrySubsystem.setDefaultCommand(new TelemetryCommand(telemetrySubsystem));
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
                .whenPressed(new RaiseWrist(wrist))
                .toggleWhenPressed(new ExtendCommand(extend), new RetractCommand(extend));

        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new Reject(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new CollectSample(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ToggleAlliance(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new Reject(intake));
//        operatorOp.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new CancelCommand(intake, pass, liftSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new SpitCommand(intake, wrist));

        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new SwingArmUpCommand(swingArmSubsystem), new SwingArmDownCommand(swingArmSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new SwingArmMidCommand(swingArmSubsystem));

        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LowerWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new RaiseWrist(wrist));

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new SwingArmDownCommand(swingArmSubsystem));
//

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new Score(swingArmSubsystem, liftSubsystem, claw, intake));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new IntakeToReadyForTopScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw));
        CommandScheduler.getInstance().schedule(new ExtendCommand(extend), new ZeroExtensionCommand(extend), new RetractCommand(extend));

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
