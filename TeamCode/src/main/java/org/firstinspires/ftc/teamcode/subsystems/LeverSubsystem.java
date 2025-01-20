package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class LeverSubsystem extends SubsystemBase {
    private Servo lever;

    public LeverSubsystem(Servo lever){
        this.lever = lever;
    }

    public void leverUp(){
        lever.setPosition(0);
    }

    public void leverDown(){
        lever.setPosition(0.25);
    }
    public void leverClear(){
        lever.setPosition(0.125);
    }


}
