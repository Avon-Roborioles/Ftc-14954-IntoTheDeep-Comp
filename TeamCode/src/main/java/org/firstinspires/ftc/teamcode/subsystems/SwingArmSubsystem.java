package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwingArmSubsystem extends SubsystemBase {
    private Servo SwingArmServo;
    private double TopPos = 0.7;
    private double MidPos = 0.4;
    private double ParkPos = 0.7;
    private TouchSensor touch;

    public SwingArmSubsystem(Servo SwingArmServo, TouchSensor touch) {
        this.SwingArmServo = SwingArmServo;
        this.touch = touch;
        this.SwingArmServo.setDirection(Servo.Direction.FORWARD);
    }
    public void up() {
        SwingArmServo.setPosition(TopPos);
    }
    public boolean isDown(){return !touch.isPressed();}
    public void down() {
        SwingArmServo.setPosition(0.01);
    }
    public void mid() { SwingArmServo.setPosition(MidPos);}
    public void park() { SwingArmServo.setPosition(ParkPos);}


    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Swing Arm Down", isDown());
    }

}

//TYPOOOOOOOOOOOOO