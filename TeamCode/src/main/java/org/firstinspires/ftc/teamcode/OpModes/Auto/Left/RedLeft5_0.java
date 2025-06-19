package org.firstinspires.ftc.teamcode.OpModes.Auto.Left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Left 5_0", group = "Auto")
public class RedLeft5_0 extends Left5_0{
    @Override
    public void SetAllianceColor() {
        RedAlliance = true;
    }

}
