package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp
@Disabled
public class Intake_Test extends CommandOpMode {
    private IntakeSubsystem subsystem;
    private GamepadEx gamepad;
    @Override
    public void initialize() {
        subsystem = new IntakeSubsystem(hardwareMap.get(DcMotor.class, "Intake"),
                hardwareMap.get(ColorSensor.class, "intakeColor"),
                hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"),
                hardwareMap.get(DistanceSensor.class, "intakeDistance"),
                hardwareMap.get(ServoImplEx.class, "allianceColor"), true);
        gamepad = new GamepadEx(gamepad1);

//        gamepad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new EjectCommand(subsystem, boxx));
//        gamepad.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new CancelCommand(subsystem));
//        gamepad.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new ToggleAlliance(subsystem));
//        gamepad.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new CollectSample(subsystem, wrist));
//        subsystem.rainbowlight();


    }
}