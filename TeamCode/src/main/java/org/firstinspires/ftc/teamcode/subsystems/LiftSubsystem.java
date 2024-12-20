package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem.liftPosition.TOP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private Motor liftMotor = null;
    private TouchSensor touch;
    private int topPosition = -3000;
    private int topBarPosition = -2200;
    private int bottomBarPosition = -600;
    private int swingarmClearPosition = -500;
    private double joystickSensitivity = 10;
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
        BOTTOMBAR,
        SWINGARMCLEAR

    }
    public liftPosition liftPos;

    public LiftSubsystem (Motor liftMotor, TouchSensor touch) {
        this.liftMotor = liftMotor;
        this.touch = touch;
        this.liftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.setInverted(true);
        this.liftMotor.setPositionTolerance(200);
        this.liftMotor.setPositionCoefficient(0.25);
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
    public void setBottomBarPosition () {
        liftMotor.setTargetPosition(bottomBarPosition);
        liftTargetPosition = bottomBarPosition;
        liftMotor.set(power);
        liftPos = liftPosition.BOTTOMBAR;
    }
    public void setSwingarmClearPosition() {
        liftMotor.setTargetPosition(swingarmClearPosition);
        liftTargetPosition = swingarmClearPosition;
        liftMotor.set(power);
        liftPos = liftPosition.SWINGARMCLEAR;
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
    }

    @Override
    public void periodic(){
        liftMotor.setTargetPosition((int) liftTargetPosition);
        liftMotor.set(Math.abs(0.05));
    }
}
