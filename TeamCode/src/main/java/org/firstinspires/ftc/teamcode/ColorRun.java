package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorRun extends LinearOpMode {
    private final org.firstinspires.ftc.teamcode.ColorTest2 colorTest = new ColorTest2();
    @Override
    public void runOpMode() throws InterruptedException {
        colorTest.initColorSensor(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            colorTest.getTelemetry(telemetry);
        }
    }

}
