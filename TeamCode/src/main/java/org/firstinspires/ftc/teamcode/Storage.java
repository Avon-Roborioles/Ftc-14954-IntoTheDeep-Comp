package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Storage  extends CommandOpMode {
    ToggleButtonReader d_up, d_down, d_left, d_right, a;
    double currentMenuY = 0;
    double currentMenuX = 1;
    double menuMaxY = 2;
    double menuMaxX = 2;
    double menuMinY = 0;
    double menuMinX = 1;
    String currentSet = null;
    public static class memory {
        public static String lastAuto = null;
        public static Pose lastPose = new Pose(0, 0, PI);
        public static Pose scorePose = new Pose(-57 ,-54 , PI/4 );
    }

    public void initPoseSelect(GamepadEx driverOp){
        d_up = new ToggleButtonReader(driverOp, GamepadKeys.Button.DPAD_UP);
        d_down = new ToggleButtonReader(driverOp, GamepadKeys.Button.DPAD_DOWN);
        d_left = new ToggleButtonReader(driverOp, GamepadKeys.Button.DPAD_LEFT);
        d_right = new ToggleButtonReader(driverOp, GamepadKeys.Button.DPAD_RIGHT);
        a = new ToggleButtonReader(driverOp, GamepadKeys.Button.A);
    }

    public void runPoseSelect( Telemetry telemetry){
        telemetry.clear();
        d_up.getState();
        d_down.getState();
        d_left.getState();
        d_right.getState();
        a.getState();
        if(d_up.isDown()){
            currentMenuY++;
        }else if(d_down.isDown()){
            currentMenuY--;
        }
        if(d_left.isDown()){
            currentMenuX--;
        }else if(d_right.isDown()){
            currentMenuX++;
        }
        if(currentMenuY > menuMaxY){
            currentMenuY = menuMinY;
        }else if(currentMenuY < menuMinY){
            currentMenuY = menuMaxY;
        }
        if(currentMenuX > menuMaxX){
            currentMenuX = menuMinX;
        }else if(currentMenuX < menuMinX){
            currentMenuX = menuMaxX;
        }


        switch ((int) currentMenuY){
            case 0:
                telemetry.addLine("Option: Use Current Pose");
                telemetry.addData("Current Set", currentSet);
                telemetry.addData("Y", currentMenuY);
                telemetry.addData("X", currentMenuX);
                telemetry.update();
                break;
            case 1:
                if(currentMenuX == 1){
                    if (a.isDown()){
                       memory.lastPose = new Pose(-16.5 - 8.1875, -2.5 - 8.5625, PI);
                       memory.scorePose = new Pose(-57 ,-54 , PI/4 );
                       currentSet = "3-1";
                    }
                    telemetry.addLine("Option: 3-1");
                } else {
                    if (a.isDown()){
                        memory.lastPose = new Pose(-16.5 - 8.1875, -2.5 - 8.5625, PI);;
                        memory.scorePose = new Pose(-57 ,-54 , PI/4 );
                        currentSet = "4-0";
                    }
                    telemetry.addLine("Option: 4-0");
                }
                telemetry.addData("Current Set", currentSet);
                telemetry.addData("Y", currentMenuY);
                telemetry.addData("X", currentMenuX);
                telemetry.update();
                break;
            case 2:
                if(currentMenuX == 1){
                    if (a.isDown()){
                        memory.lastPose = new Pose(60, -60, PI);
                        memory.scorePose = new Pose(-57 ,-54 , PI/4 );
                        currentSet = "Grab 1";
                    }
                    telemetry.addLine("Option: Grab 1");
                } else {
                    if (a.isDown()){
                        memory.lastPose = new Pose( 8.5625-0.75 , -34, -PI/2);
                        memory.scorePose = new Pose(-57 ,-54 , PI/4 );
                        currentSet = "Grab 2";
                    }
                    telemetry.addLine("Option: Grab 2");
                }
                telemetry.addData("Current Set", currentSet);
                telemetry.addData("Y", currentMenuY);
                telemetry.addData("X", currentMenuX);
                telemetry.update();
                break;

        }


        /*
          0   Use Current Pose
          1   L 3-1    |  L 4-0
          2   R G1     |  R G2
         */
    }


}
