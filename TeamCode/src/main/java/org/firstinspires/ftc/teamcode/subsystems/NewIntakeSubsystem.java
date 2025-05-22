package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class NewIntakeSubsystem extends SubsystemBase {
    private DcMotorEx motor;
    private ColorSensor colorSensor, rampSensor;


    public NewIntakeSubsystem(DcMotorEx motor, ColorSensor colorSensor, ColorSensor rampSensor) {
        this.motor = motor;
        this.colorSensor = colorSensor;
        this.rampSensor = rampSensor;

        this.motor.setDirection(DcMotor.Direction.FORWARD);

    }

//    @Override
//    public void periodic() {
//        if (RedAlliance) {
//            allianceColor.setPosition(0.28);
//        } else {
//            allianceColor.setPosition(0.61);
//        }
//    }


    //1 is too fast
    public void runMotor() {motor.setPower(1);}

    public void rejectMotor() {motor.setPower(-0.9);
    }
    public boolean isStalled(){
        return motor.getCurrent(CurrentUnit.MILLIAMPS)>5500;
    }

    public void stopMotor() {
        motor.setPower(0);

    }
    public boolean inRamp(){
        return rampSensor.red()+rampSensor.blue()+rampSensor.green()>1000;
    }

//    }
    public boolean hasSample() {
        return colorSensor.green()>300;
//        return ((colorSensor.blue()+colorSensor.red()+colorSensor.green())>1500);
    }
    public boolean hasBlueSample() {
        return colorSensor.blue()>300 && !isColorSensorRed();
//        return ((colorSensor.blue()+colorSensor.red()+colorSensor.green())>1500);
    }
    public boolean hasRedSample() {
        return colorSensor.green()>300 && !isColorSensorBlue();
//        return ((colorSensor.blue()+colorSensor.red()+colorSensor.green())>1500);
    }
    public boolean correctColor(boolean RedAlliance){
        if (RedAlliance){
            return !isColorSensorBlue();
        }else {
            return !isColorSensorRed();
        }
    }
    //R,G,B
    //Blue: R:150-200, G:300-400, B:700-2000
    //Red: R:700-2000, G:350-450, B:100-200 (note Blue spikes when close to sensor but red and green are both way greater)
    //Yellow: R:1000-2000, G:1000-2000, B:350-400
    public boolean isSampleRedOrYellow(){
           return colorSensor.red()>600;
    }
    public boolean isSampleBlueOrYellow(){
        return ((!isSampleRedOrYellow() || ((colorSensor.green()+colorSensor.red()-colorSensor.blue())> 2100) && (colorSensor.blue()<400 && colorSensor.blue()>200) )&& hasSample());
//        return (!isSampleRedOrYellow() || colorSensor.green()+colorSensor.blue()>1000) && hasSample();
    }

    public boolean isColorSensorRed() {return colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green();}

    public boolean isColorSensorBlue() {return colorSensor.blue() > colorSensor.red()&& colorSensor.blue()>150;}

    public boolean isColorSensorYellow() {
        return !isColorSensorBlue() && !isColorSensorRed() && colorSensor.green()> 200;
    }



    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Motor running", motor.getPower());
        telemetry.addData("red or Yellow", isSampleRedOrYellow());
        telemetry.addData("blue or Yellow", isSampleBlueOrYellow());
        telemetry.addData("Red Sensor", colorSensor.red());
        telemetry.addData("Blue Sensor", colorSensor.blue());
        telemetry.addData("Green Sensor", colorSensor.green());
        telemetry.addData("Red Sample", isColorSensorRed());
        telemetry.addData("Blue Sample", isColorSensorBlue());
        telemetry.addData("Yellow Sample", isColorSensorYellow());
        telemetry.addData("motorCurrent", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();



    }

    }

