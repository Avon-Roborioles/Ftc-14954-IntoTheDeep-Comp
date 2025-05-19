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
        return motor.getCurrent(CurrentUnit.MILLIAMPS)>6500;
    }

    public void stopMotor() {
        motor.setPower(0);

    }
    public boolean inRamp(){
        return rampSensor.red()+rampSensor.blue()+rampSensor.green()>2000;
    }

//    }
    public boolean hasSample() {
        return (getBlue()+getRed()+getGreen()>1500);
    }

    public double getRed() {
//        return colorSensor1.red()+ colorSensor2.red();}
        return colorSensor.red();}
    public double getBlue() {
//        return colorSensor1.blue()+ colorSensor2.blue();}
     return colorSensor.blue();}
    public double getGreen() {
//        return colorSensor1.green()+ colorSensor2.green();}
     return colorSensor.green();}

    public boolean isColorSensorRed() {return getRed() > getBlue() && getRed() > getGreen();}

    public boolean isColorSensorBlue() {return getBlue() > getRed()&& getBlue()>200;}

    public boolean isColorSensorYellow() {
        return !isColorSensorBlue() && !isColorSensorRed() && getGreen() > 200;
    }



    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Motor running", motor.getPower());
        telemetry.addData("Red Sensor", getRed());
        telemetry.addData("Blue Sensor", getBlue());
        telemetry.addData("Green Sensor", getGreen());
        telemetry.addData("Red Sample", isColorSensorRed());
        telemetry.addData("Blue Sample", isColorSensorBlue());
        telemetry.addData("Yellow Sample", isColorSensorYellow());
        telemetry.addData("motorCurrent", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();



    }

    }

