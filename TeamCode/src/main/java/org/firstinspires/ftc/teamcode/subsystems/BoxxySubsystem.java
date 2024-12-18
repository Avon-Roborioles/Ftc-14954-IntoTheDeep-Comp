package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BoxxySubsystem extends SubsystemBase {

    private DistanceSensor distanceSensor;
    public BoxxySubsystem( DistanceSensor distanceSensor) {
//        this.servo = servo;
        this.distanceSensor = distanceSensor;
    }
//    public void open() {
//        servo.setPosition(1.0);
//
//    }
//    public void close() {
//        servo.setPosition(0.5);
//    }
    public boolean haveSample() {
        return(distanceSensor.getDistance(DistanceUnit.INCH) < 2.0);
    }

    public boolean noSample(){
        return (distanceSensor.getDistance(DistanceUnit.INCH) > 2.5);
    }
    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("box distance", distanceSensor.getDistance(DistanceUnit.INCH));
    }

}
