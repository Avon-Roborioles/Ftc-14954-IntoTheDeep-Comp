package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem.liftPosition.TOP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private Motor liftMotor = null;
    private TouchSensor touch;
    private int topPosition = 2300;
    private int topBarPosition = 1600;
    private int topBarHookPosition = 1100;
    private int bottomBucketPosition = 840;
    private int swingarmClearPosition = 950;
    private double liftTargetPosition = 0;
    private double power = 1;

    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("liftEnum", liftPos);
        telemetry.addData("rawpos", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Bottom", touch.isPressed());
    }

    public enum liftPosition {
        TOP,
        BOTTOM,
        TOPBAR,
        SWINGARMCLEAR,
        BOTTOMBUCKET,
        TOPHOOK;

    }
    public liftPosition liftPos;

    public LiftSubsystem (Motor liftMotor, TouchSensor touch) {
        this.liftMotor = liftMotor;
        this.touch = touch;
        this.liftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.setInverted(true);
        this.liftMotor.setPositionTolerance(25);
        this.liftMotor.setPositionCoefficient(0.0375);
        this.liftMotor.resetEncoder();
        liftPos = liftPosition.BOTTOM;

    }
    public void setTopPosition () {
        liftMotor.setTargetPosition(topPosition);
        liftTargetPosition= topPosition;
        liftMotor.set(power);
        liftPos = liftPosition.TOP;
    }
    public void setBottomPosition () {
        liftMotor.setTargetPosition(0);
        liftTargetPosition = 0;
        liftMotor.set(power);
        liftPos = liftPosition.BOTTOM;
    }
    public void setTopBarPosition () {
        liftMotor.setTargetPosition(topBarPosition);
        liftTargetPosition = topBarPosition;
        liftMotor.set(power);
        liftPos = liftPosition.TOPBAR;
    }

    public void setSwingarmClearPosition() {
        liftMotor.setTargetPosition(swingarmClearPosition);
        liftTargetPosition = swingarmClearPosition;
        liftMotor.set(power);
        liftPos = liftPosition.SWINGARMCLEAR;
    }
    public void setBottomBucketPosition() {
        liftMotor.setTargetPosition(bottomBucketPosition);
        liftTargetPosition = bottomBucketPosition;
        liftMotor.set(power);
        liftPos = liftPosition.BOTTOMBUCKET;
    }
    public void setTopHookPosition() {
        liftMotor.setTargetPosition(topBarHookPosition);
        liftTargetPosition = topBarHookPosition;
        liftMotor.set(power);
        liftPos = liftPosition.TOPHOOK;
    }
    public void setPower (double power) {
        liftMotor.setRunMode(Motor.RunMode.RawPower);
        liftMotor.set(power);
    }
    public boolean isBusy () {
        return !liftMotor.atTargetPosition();
    }

    public void stopLift () {
        liftMotor.stopMotor();
    }

    public boolean isBottom() {
        return touch.isPressed();
    }
    public void resetEncoder() {
        if ( isBottom()){
            liftMotor.resetEncoder();
        }
        liftMotor.setRunMode(Motor.RunMode.PositionControl);
    }

    @Override
    public void periodic(){
        liftMotor.setTargetPosition((int) liftTargetPosition);
        liftMotor.set(Math.abs(0.05));
    }
}
