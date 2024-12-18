package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;

@TeleOp
@Disabled
public class BoxxyTest extends CommandOpMode {
    private BoxxySubsystem subsystem;
    private GamepadEx gamepadEx;
    @Override
    public void initialize() {
//        subsystem = new BoxxySubsystem( hardwareMap.get(Servo.class, "Boxxy"));
//        gamepadEx = new GamepadEx(gamepad1);
//        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new OpenBoxxy(subsystem));
//        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new CloseBoxxy(subsystem));

    }
}


