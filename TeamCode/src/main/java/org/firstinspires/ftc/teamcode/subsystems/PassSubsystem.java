package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PassSubsystem extends SubsystemBase {
    private DcMotorEx motor1;
    private Rev2mDistanceSensor distanceSensor;

    public PassSubsystem(DcMotorEx motor1, Rev2mDistanceSensor distanceSensor) {
        this.motor1 = motor1;
        this.distanceSensor = distanceSensor;
    }
    public void PassMotorControl(double motorPower){
        motor1.setPower(motorPower);
    }
    public void PassOn(){
        motor1.setPower(0.75);
    }
    public void PassOff(){
        motor1.setPower(0);
    }
    public boolean PassDistanceTrue(){
        return distanceSensor.getDistance(DistanceUnit.INCH) < 0.75 ;
    }

    public void getTelemetry(Telemetry telemetry) {
    }
}

