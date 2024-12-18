package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtendSubsystem extends SubsystemBase {
    private Servo servo;

    public ExtendSubsystem(Servo servo) { this.servo = servo; }

    public void extend() { servo.setPosition(0.55);}

    public void retract() { servo.setPosition(1.0);}

    public void getTelemetry(Telemetry telemetry) {

    }
}
