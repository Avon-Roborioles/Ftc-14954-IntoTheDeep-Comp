package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NewCollectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NewSpitCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NewStopCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.NomNomComand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name = "IntakeTestOpMode", group = "Test Op Modes")
public class IntakeTestOpMode extends CommandOpMode {
    private Motor frontLeft, frontRight, backLeft, backRight;
    private GamepadEx driverOp, operatorOp;
    private WristSubsystem wrist;
    private NewIntakeSubsystem intake;


    @Override
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"));

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new RaiseWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LowerWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new NewCollectCommand(intake, wrist, telemetry));
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new NewStopCommand(intake));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new NewSpitCommand(intake, wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new NomNomComand(intake, wrist, telemetry));


    }

}
