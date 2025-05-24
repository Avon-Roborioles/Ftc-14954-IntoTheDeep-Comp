package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.commands.ClawCommands.CloseClawCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommands.OpenClawCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NewCollectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NewSpitCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NewStopCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NomNomComand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.AutoLiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name = "IntakeTestOpMode", group = "Test Op Modes")
public class IntakeTestOpMode extends CommandOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight;
    private GamepadEx driverOp, operatorOp;
    private WristSubsystem wrist;
    private NewIntakeSubsystem intake;
    private ExtendSubsystem extend;
    private ServoImplEx extendservo;
    private SwingArmSubsystem swingArm;
    private LiftSubsystem lift;
    private ClawSubsystem claw;
    PwmControl.PwmRange servoRange = new PwmControl.PwmRange(799, 1500);
    private TouchSensor touch1, touch2;


    @Override
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"));
        extendservo = hardwareMap.get(ServoImplEx.class, "extension");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");
        swingArm = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        extendservo.setPwmRange(servoRange);
        extend = new ExtendSubsystem(extendservo, touch2 );
        claw = new ClawSubsystem (hardwareMap.get(Servo.class, "claw"));
        lift = new LiftSubsystem (hardwareMap.get(Motor.class, "liftMotor"), hardwareMap.get(TouchSensor.class, "liftDown"));
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new ExtendCommand(extend));
        driverOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new RetractCommand(extend));
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new RaiseWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LowerWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new NewSpitCommand(intake, wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new NewStopCommand(intake));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new NomNomComand(intake, wrist,extend, telemetry, false));
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new NomNomComand(intake, wrist, extend, telemetry, true));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new NewCollectCommand(intake, wrist, telemetry, false));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new NewCollectCommand(intake, wrist, telemetry, true));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new SwingArmUpCommand(swingArm));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new SwingArmDownCommand(swingArm));
        operatorOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new OpenClawCommand(claw));
        operatorOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new CloseClawCommand(claw));
        operatorOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new LiftTopCommand(lift));
        operatorOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftBottomCommand(lift));




    }

}
