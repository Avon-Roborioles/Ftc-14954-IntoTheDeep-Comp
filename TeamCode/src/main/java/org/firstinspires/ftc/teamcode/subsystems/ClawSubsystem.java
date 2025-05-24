package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private Servo claw;
    public ClawSubsystem(Servo claw) {
        this.claw = claw;
    }
    public void open(){claw.setPosition(0.0);}
    public void close(){claw.setPosition(0.5);}
}
