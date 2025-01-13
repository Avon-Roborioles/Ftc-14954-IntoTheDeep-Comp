package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor motor;
    private ColorSensor colorSensor1, colorSensor2;
    private RevBlinkinLedDriver blinkin;
    private boolean RedAlliance = false;
    private DistanceSensor distanceSensor;
    private ServoImplEx allianceColor;
    private BoxxySubsystem box;

    public IntakeSubsystem(DcMotor motor, ColorSensor colorSensor1, ColorSensor colorSensor2, RevBlinkinLedDriver blinkin,
                           DistanceSensor distanceSensor, ServoImplEx allianceColor, boolean RedAlliance) {
        this.motor = motor;
        this.colorSensor1 = colorSensor1;
        this.colorSensor2 = colorSensor2;
        this.blinkin = blinkin;
        this.distanceSensor = distanceSensor;
        this.allianceColor = allianceColor;
        this.box = box;
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        this.RedAlliance = RedAlliance;
        // start as blue alliance
        if(!RedAlliance){
            this.allianceColor.setPosition(0.61);
            this.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }else {
            this.allianceColor.setPosition(0.28);
            this.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    @Override
    public void periodic() {
        if (RedAlliance) {
            allianceColor.setPosition(0.28);
        } else {
            allianceColor.setPosition(0.61);
        }
    }

    public void runMotor() {motor.setPower(-0.8);}

    public void rejectMotor() {motor.setPower(0.8);}

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

    public void stopMotor() {
        motor.setPower(0);
    }

    public boolean hasSample() {return (distanceSensor.getDistance(DistanceUnit.INCH) < 2.5);}

    public double getRed() {return colorSensor1.red()+ colorSensor2.red();}
    public double getBlue() {return colorSensor1.blue()+ colorSensor2.blue();}
    public double getGreen() {return colorSensor1.green()+ colorSensor2.green();}

    public boolean isColorSensorRed() {return getRed() > getBlue() && !(getGreen() /getBlue() >2);}

    public boolean isColorSensorBlue() {return getBlue() > getRed();}

    public boolean isColorSensorYellow() {
        return !isColorSensorBlue() && !isColorSensorRed() && getGreen() > 200;
    }

    public void changeAlliance() {
        this.RedAlliance = !getRedAlliance();
        if (RedAlliance) {
            allianceColor.setPosition(0.28);
        } else {
            allianceColor.setPosition(0.61);
        }
    }

    public void setRedAlliance() {
        RedAlliance = true;
    }
    public void redAllianceLight(){allianceColor.setPosition(0.28);}
    public void blueAllianceLight(){allianceColor.setPosition(0.61);}

    public void setBlueAlliance() {
        RedAlliance = false;
    }

    public void getTelemetry(Telemetry telemetry) {
        telemetry.addData("Motor running", motor.getPower());
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Red Sensor 1", colorSensor1.red());
        telemetry.addData("Blue Sensor 1", colorSensor1.blue());
        telemetry.addData("Green Sensor 1", colorSensor1.green());
        telemetry.addData("Red Sensor 2", colorSensor2.red());
        telemetry.addData("Blue Sensor 2", colorSensor2.blue());
        telemetry.addData("Green Sensor 2", colorSensor2.green());
        telemetry.addData("Red Sample", isColorSensorRed());
        telemetry.addData("Blue Sample", isColorSensorBlue());
        telemetry.addData("Yellow Sample", isColorSensorYellow());


        if (RedAlliance) {
            telemetry.addData("Alliance", "Red");
        } else {
            telemetry.addData("Alliance", "Blue");
        }
    }

        public boolean getRedAlliance() {
            return RedAlliance;
        }
    }

