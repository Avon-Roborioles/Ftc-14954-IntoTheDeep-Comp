package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem extends SubsystemBase {
    private Servo servo;
    public WristSubsystem(Servo servo) {
        this.servo = servo;

    }
    public void up() {
        servo.setPosition(0.36);
    }
    public void down() {
        servo.setPosition(0);
    }


    public void getTelemetry(Telemetry telemetry) {
    }

}
