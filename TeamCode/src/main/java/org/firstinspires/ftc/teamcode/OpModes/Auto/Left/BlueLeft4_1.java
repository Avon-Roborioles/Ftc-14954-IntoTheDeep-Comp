package org.firstinspires.ftc.teamcode.OpModes.Auto.Left;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Left 4_1", group = "Auto")
public class BlueLeft4_1 extends Left4_1{
    @Override
    public void SetAllianceColor() {
        RedAlliance = false;
    }

}
