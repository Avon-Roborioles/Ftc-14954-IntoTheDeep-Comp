package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private CRServo intakeRoller;
    private boolean DistanceSensorCooked = false;

    public IntakeSubsystem(DcMotor motor, ColorSensor colorSensor1, ColorSensor colorSensor2, RevBlinkinLedDriver blinkin,
                           DistanceSensor distanceSensor, ServoImplEx allianceColor, boolean RedAlliance, CRServo intakeRoller) {
        this.motor = motor;
        this.colorSensor1 = colorSensor1;
        this.colorSensor2 = colorSensor2;
        this.blinkin = blinkin;
        this.distanceSensor = distanceSensor;
        this.allianceColor = allianceColor;
        this.box = box;
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        this.RedAlliance = RedAlliance;
        this.intakeRoller = intakeRoller;
        this.intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD);
        // start as blue alliance
        if(!RedAlliance){
            this.allianceColor.setPosition(0.61);
            this.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }else {
            this.allianceColor.setPosition(0.28);
            this.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        DistanceSensorCooked = false;
    }

    @Override
    public void periodic() {
        if (RedAlliance) {
            allianceColor.setPosition(0.28);
        } else {
            allianceColor.setPosition(0.61);
        }
    }

    public void runMotor() {motor.setPower(-1);
    intakeRoller.setPower(1);}

    public void rejectMotor() {motor.setPower(1);
    intakeRoller.setPower(-1);}

    public boolean IsIntakeDistanceSensorCooked() {
        return DistanceSensorCooked;
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

    public void stopMotor() {
        motor.setPower(0);
        intakeRoller.setPower(0);
    }
    public void IntakeFailLight() {
        allianceColor.setPosition(0.722); // Violet
    }
    public void PassFailLight() {
        allianceColor.setPosition(0.388); // Yellow
    }
    public void BoxFailLight() {
        allianceColor.setPosition(0.5); // Green
    }

    public boolean hasSample() {
        if (distanceSensor.getDistance(DistanceUnit.INCH) > 10) {
            DistanceSensorCooked = true;
            IntakeFailLight();
        }
        if(DistanceSensorCooked){
            return (getBlue()+getRed()+getGreen()>1000);
        }else {
            return (distanceSensor.getDistance(DistanceUnit.INCH) < 3);
        }
    }

    public double getRed() {
        return colorSensor1.red()+ colorSensor2.red();}
    public double getBlue() {
        return colorSensor1.blue()+ colorSensor2.blue();}
    public double getGreen() {
        return colorSensor1.green()+ colorSensor2.green();}

    public boolean isColorSensorRed() {return getRed() > getBlue() && getRed() > getGreen();}

    public boolean isColorSensorBlue() {return getBlue() > getRed()&& getBlue()>200;}

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
        telemetry.addData("Red Sensor", getRed());
        telemetry.addData("Blue Sensor", getBlue());
        telemetry.addData("Green Sensor", getGreen());
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

