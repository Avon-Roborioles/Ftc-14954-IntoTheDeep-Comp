package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtendSubsystem extends SubsystemBase {
    private Servo servo;
    private TouchSensor touch;

    public ExtendSubsystem(Servo servo, TouchSensor touch) {  this.servo = servo; this.touch = touch; }

    public void extend() { servo.setPosition(0.45);}

    public void retract() { servo.setPosition(0.96);}
    public boolean retracted(){return touch.isPressed();}
    public void setPosition(double position){servo.setPosition(position);}


    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Extend Retracted", touch.isPressed());

    }
}
