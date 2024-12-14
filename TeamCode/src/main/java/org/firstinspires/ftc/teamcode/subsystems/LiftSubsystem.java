package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem.liftPosition.TOP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private Motor liftMotor = null;
    private int topPosition = -3000;
    private int topBarPosition = -100;
    private double joystickSensitivity = 10;
    private double liftTargetPosition = 0;
    private Telemetry telemetry;
    private double power = 1;

    public enum liftPosition {
        TOP,
        BOTTOM,
        TOPBAR

    }
    public liftPosition liftPos;

    public LiftSubsystem (Motor liftMotor, Telemetry telemetry) {
        this.liftMotor = liftMotor;
        this.telemetry = telemetry;
        this.liftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.liftMotor.setInverted(true);
        this.liftMotor.setPositionTolerance(100);
        this.liftMotor.resetEncoder();
        liftPos = liftPosition.BOTTOM;

    }
    public void setTopPosition () {
        liftMotor.setTargetPosition(topPosition);
//        liftTargetPosition= topPosition;
        liftMotor.set(power);
        liftPos = liftPosition.TOP;
    }
    public void setBottomPosition () {
        liftMotor.setTargetPosition(0);
//        liftTargetPosition = 0;
        liftMotor.set(power);
        liftPos = liftPosition.BOTTOM;
    }
    public void setTopBarPosition () {
        liftMotor.setTargetPosition(-300);
//        liftTargetPosition = topBarPosition;
        liftMotor.set(power);
        liftPos = liftPosition.TOPBAR;
    }
    public boolean isBusy () {
        return !liftMotor.atTargetPosition();
    }

    public void stopLift () {
        liftMotor.stopMotor();
    }
    public void runLiftJoystick(double input){
        power = input;
        liftTargetPosition = liftMotor.getCurrentPosition() + (-input * joystickSensitivity);
        if (liftTargetPosition < topPosition) {
            liftTargetPosition = topPosition;
        }else if(liftTargetPosition > 0) {
            liftTargetPosition = 0;
        }
        liftMotor.setTargetPosition((int) liftTargetPosition);
        liftMotor.set(Math.abs(power));
        getLiftTelemetry();
    }
    public void getLiftTelemetry () {
        telemetry.addData("liftEnum", liftPos);
        telemetry.addData("rawpos", liftMotor.getCurrentPosition());
        telemetry.update();

    }
//    @Override
//    public void periodic(){
//        liftMotor.setTargetPosition((int) liftTargetPosition);
//        liftMotor.set(Math.abs(power));
//    }
}
