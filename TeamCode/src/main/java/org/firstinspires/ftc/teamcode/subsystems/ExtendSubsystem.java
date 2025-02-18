package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtendSubsystem extends SubsystemBase {
    private ServoImplEx servo;

    private TouchSensor touch;

    public ExtendSubsystem(Servo servo, TouchSensor touch) {
        this.servo = (ServoImplEx) servo; this.touch = touch;
    }

    public void extend() { servo.setPosition(0.45);}

    public void retract() { servo.setPosition(0.955);}
    public boolean retracted(){return touch.isPressed();}
    public void setPosition(double position){servo.setPosition(position);}
    public void disable(){
        servo.setPwmDisable();
    }
    public void enable(){
        servo.setPwmEnable();
    }


    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Extend Retracted", touch.isPressed());

    }
}
