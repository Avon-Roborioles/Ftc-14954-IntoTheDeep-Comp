package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PassSubsystem extends SubsystemBase {
    private DcMotorEx motor1;
    private Rev2mDistanceSensor distanceSensor;
    private IntakeSubsystem intake;
    private boolean DistanceSensorCooked = false;

    public PassSubsystem(DcMotorEx motor1, Rev2mDistanceSensor distanceSensor, IntakeSubsystem intake) {
        this.motor1 = motor1;
        this.motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.distanceSensor = distanceSensor;
        this.intake = intake;
        DistanceSensorCooked = false;
    }
    public void PassMotorControl(double motorPower){
        motor1.setPower(motorPower);
    }
    public void PassOn(){
        motor1.setPower(1);
    }
    public void PassOff(){
        motor1.setPower(0);
    }
    public boolean IsPassDistanceSensorCooked() {
        return DistanceSensorCooked;
    }
    public boolean PassDistanceTrue(){
        if (distanceSensor.getDistance(DistanceUnit.INCH ) >10){
            DistanceSensorCooked = true;
//            intake.PassFailLight();

        }
        return distanceSensor.getDistance(DistanceUnit.INCH) < 2 ;
    }

    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("pass distance", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}

