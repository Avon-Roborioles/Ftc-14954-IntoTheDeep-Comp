package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtendSubsystem extends SubsystemBase {
    private DcMotorEx motor;
    private int maxPosition = 450;
    private double power = 0.5;
    private int target= 0;
    private TouchSensor touch;

    public ExtendSubsystem(TouchSensor touch, DcMotorEx motor) {
        this.motor = motor;
        this.motor.setTargetPosition(0);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setPositionPIDFCoefficients(13);
        this.motor.setTargetPositionTolerance(5);
        this.touch = touch;
    }

    public void extend() {
        motor.setTargetPosition(maxPosition);
        target = maxPosition;
    }
    public void clear(){
        motor.setTargetPosition(190);
        target = 190;

    }

    public void retract() {
        motor.setTargetPosition(0);
        motor.setPower(power);
        target = 0;
    }
    public boolean retracted(){return touch.isPressed();}

    public void setPosition(double position){
        motor.setTargetPosition((int) (maxPosition*position));
        target = (int) (maxPosition*position);
    }
    public boolean isBusy(){
        return motor.isBusy();
    }
    public void zero() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Extend Retracted", touch.isPressed());
        telemetry.addData("Extend Motor Position", motor.getCurrentPosition());
        telemetry.addData("Extend Target Position", target);
    }
    public void setPower (double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }
    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void periodic(){
        motor.setTargetPosition(target);
        motor.setPower(power);
    }
}
