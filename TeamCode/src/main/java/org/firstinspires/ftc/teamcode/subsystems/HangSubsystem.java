package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class HangSubsystem extends SubsystemBase {
    private Motor motor;
    public HangSubsystem(Motor motor){
        this.motor = motor;
        this.motor.setInverted(false);
        this.motor.setRunMode(Motor.RunMode.VelocityControl);
        this.motor.setVeloCoefficients(0, 1.5, 0);
        this.motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void up(){motor.set(1);}
    public void down(){motor.set(-1);}
    public void stop(){motor.set(0);}
    public void setHangPower(double power){motor.set(power);}


}
