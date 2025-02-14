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
        servo.setPosition(0.90);
    }
    public void down() {
        servo.setPosition(0.155);
    }
    public void test() {
        servo.getPosition();
    }
    public void middle() { servo.setPosition(0.4);}
    public void ClearCenter() { servo.setPosition(0.425);}

    public void getTelemetry(Telemetry telemetry) {
    }

}
