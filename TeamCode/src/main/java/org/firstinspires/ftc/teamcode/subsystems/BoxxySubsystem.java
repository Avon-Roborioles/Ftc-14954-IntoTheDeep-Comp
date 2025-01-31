package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BoxxySubsystem extends SubsystemBase {

    private DistanceSensor distanceSensor;
    private IntakeSubsystem intake;
    private boolean DistanceSensorCooked = false;
    public BoxxySubsystem( DistanceSensor distanceSensor, IntakeSubsystem intake) {
        this.distanceSensor = distanceSensor;
        this.intake = intake;
        DistanceSensorCooked = false;
    }
    public boolean IsBoxDistanceSensorCooked() {
        return DistanceSensorCooked;
    }
    public boolean haveSample() {
        if (distanceSensor.getDistance(DistanceUnit.INCH ) >10){
            DistanceSensorCooked = true;
            intake.BoxFailLight();

        }
        return(distanceSensor.getDistance(DistanceUnit.INCH) < 2.5);
    }

    public boolean noSample(){
        if (distanceSensor.getDistance(DistanceUnit.INCH ) >10){
            DistanceSensorCooked = true;
            intake.BoxFailLight();

        }
        return (distanceSensor.getDistance(DistanceUnit.INCH) > 2.5);
    }
    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("box distance", distanceSensor.getDistance(DistanceUnit.INCH));
    }

}
