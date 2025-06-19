package org.firstinspires.ftc.teamcode.OpModes.Auto.Left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Left 4_1", group = "Auto")
public class RedLeft4_1 extends Left4_1{
    @Override
    public void SetAllianceColor() {
        RedAlliance = true;
    }

}
