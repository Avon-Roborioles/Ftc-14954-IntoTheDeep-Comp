package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class ColorRun extends LinearOpMode {
    private final ColorTest2 colorTest = new ColorTest2();
    @Override
    public void runOpMode() throws InterruptedException {
        colorTest.initColorSensor(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            colorTest.getTelemetry(telemetry);
        }
    }

}
