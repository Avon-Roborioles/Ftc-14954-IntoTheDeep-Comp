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
    private RevBlinkinLedDriver blinkin;
    private ServoImplEx allianceColor;
    private boolean RedAlliance = false;
    private boolean SkippedLastSample = false;


    public NewIntakeSubsystem(DcMotorEx motor, ColorSensor colorSensor, ColorSensor rampSensor, RevBlinkinLedDriver blinkin, ServoImplEx allianceColor, Boolean RedAlliance) {
        this.motor = motor;
        this.colorSensor = colorSensor;
        this.rampSensor = rampSensor;
        this.blinkin = blinkin;
        this.allianceColor = allianceColor;
        this.RedAlliance = RedAlliance;
        if(!RedAlliance){
            this.allianceColor.setPosition(0.61);
            this.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }else {
            this.allianceColor.setPosition(0.28);
            this.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        this.motor.setDirection(DcMotor.Direction.REVERSE);

        SkippedLastSample = false;
    }
    public NewIntakeSubsystem(DcMotorEx motor, ColorSensor colorSensor, ColorSensor rampSensor, RevBlinkinLedDriver blinkin, ServoImplEx allianceColor) {
        this.motor = motor;
        this.colorSensor = colorSensor;
        this.rampSensor = rampSensor;
        this.blinkin = blinkin;
        this.allianceColor = allianceColor;
        this.motor.setDirection(DcMotor.Direction.REVERSE);

        SkippedLastSample = false;
    }



    public void runMotor() {motor.setPower(1);}

    public void rejectMotor() {motor.setPower(-0.75);
    }
    public boolean isStalled(){
        return motor.getCurrent(CurrentUnit.MILLIAMPS)>8000;
    }

    public void stopMotor() {
        motor.setPower(0);

    }
    public void redlight() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void bluelight() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void yellowlight() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void rainbowlight() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }
    public void redAllianceLight(){allianceColor.setPosition(0.28);}
    public void blueAllianceLight(){allianceColor.setPosition(0.61);}
    public double getRed() {
//        return colorSensor1.red()+ colorSensor2.red();}
        return rampSensor.red();}
    public double getBlue() {
//        return colorSensor1.blue()+ colorSensor2.blue();}
        return rampSensor.blue();}
    public double getGreen() {
//        return colorSensor1.green()+ colorSensor2.green();}
        return rampSensor.green();}

    public boolean isRampSensorRed() {return getRed() > getBlue() && getRed() > getGreen();}

    public boolean isRampSensorBlue() {return getBlue() > getRed()&& getBlue()>200;}

    public boolean isRampSensorYellow() {
        return !isRampSensorBlue() && !isRampSensorRed() && getGreen() > 200;
    }
    public boolean inRamp(){
        return rampSensor.red()+rampSensor.blue()+rampSensor.green()>1000;
    }

//    }
    public boolean hasSample() {
        return colorSensor.green()>300;
//        return ((colorSensor.blue()+colorSensor.red()+colorSensor.green())>1500);
    }
    //R,G,B
    //Blue: R:150-200, G:300-400, B:700-2000
    //Red: R:700-2000, G:350-450, B:100-200 (note Blue spikes when close to sensor but red and green are both way greater)
    //Yellow: R:1000-2000, G:1000-2000, B:350-400
    public boolean isSampleRedOrYellow(){
           return colorSensor.red()>600 && hasSample();

    }
    public boolean isSampleBlueOrYellow(){
//        return ((!isSampleRedOrYellow() || ((colorSensor.green()+colorSensor.red()-colorSensor.blue())> 2100) && (colorSensor.blue()<400 && colorSensor.blue()>200) )&& hasSample());
//        return (!isSampleRedOrYellow() || colorSensor.green()+colorSensor.blue()>1000) && hasSample();
        return  (!isSampleRedOrYellow() || (colorSensor.alpha() > 1100)) && hasSample();
    }

    public boolean isColorSensorRed() {return colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green();}

    public boolean isColorSensorBlue() {return colorSensor.blue() > colorSensor.red()&& colorSensor.blue()>150;}

    public boolean isColorSensorYellow() {
        return !isColorSensorBlue() && !isColorSensorRed() && colorSensor.green()> 200;
    }

    public boolean correctSample(){
        if(RedAlliance){
            return isSampleRedOrYellow();
        }else{
            return isSampleBlueOrYellow();
        }
    }

    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Motor running", motor.getPower());
        telemetry.addData("red or Yellow", isSampleRedOrYellow());
        telemetry.addData("blue or Yellow", isSampleBlueOrYellow());
        telemetry.addData("Red Sensor", colorSensor.red());
        telemetry.addData("Blue Sensor", colorSensor.blue());
        telemetry.addData("Green Sensor", colorSensor.green());
        telemetry.addData("alpha", colorSensor.alpha());
        telemetry.addData("Red Sample", isColorSensorRed());
        telemetry.addData("Blue Sample", isColorSensorBlue());
        telemetry.addData("Yellow Sample", isColorSensorYellow());
        telemetry.addData("motorCurrent", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.update();



    }
    public void setSkipLastSample(boolean skippedLastSample) {
        SkippedLastSample = skippedLastSample;
    }
    public boolean getSkipLastSample() {
        return SkippedLastSample;
    }

    public void setRedAlliance() {
        RedAlliance = true;
    }
    public boolean getRedAlliance() {
        return RedAlliance;
    }
    public void setBlueAlliance() {
        RedAlliance = false;
    }
    public void changeAlliance() {
        this.RedAlliance = !getRedAlliance();
        if (RedAlliance) {
            allianceColor.setPosition(0.28);
        } else {
            allianceColor.setPosition(0.61);
        }
    }

}

