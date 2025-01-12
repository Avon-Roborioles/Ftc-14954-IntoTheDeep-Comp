package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static java.lang.Math.PI;

import com.pedropathing.localization.Pose;

public class PoseList {
    

    // 46 6/8 - 1/2 robot width

    public static double HalfChassisWidth = 8.375;
    public static double HalfChassisLength = 8.1875;
    public static double HalfTilePuzzle = 0.75/2;
    public static Pose LStartBucket = new Pose( 0,0, 0);
    public static Pose LStartBar = new Pose( 0,-29.375, -PI);
    public static Pose LScore = new Pose(10, 17, -PI/4);
    public static Pose LBar = new Pose(20, -29.375, -PI);
    public static Pose LGrab1 = new Pose(22, 3, PI/6);
    public static Pose LGrab2 = new Pose(22, 12, PI/6);
    public static Pose LGrab3 = new Pose(24, 19, PI/4);
    public static Pose LGrab3Mid = new Pose(19.5, 17, PI/4);
    public static Pose LForward1 = new Pose(24, 5 , PI/6);
    public static Pose LForward2 = new Pose(24, 13  , PI/6);
    public static Pose LForward3 = new Pose(27, 21, PI/4);
    public static Pose LPark = new Pose(55, -15, PI/2);
    public static Pose LParkMid = new Pose(55, 17, PI/2);

    public static Pose RLStartBar = new Pose(0 ,0, 0);
    public static Pose RLBar = new Pose(0, -36, 0);
    public static Pose RLStartBucket = new Pose( -46 + HalfChassisWidth,-70.125 + HalfChassisLength, PI/2);
    public static Pose RLScore = new Pose(-61 + 5.75,-61 + 5.75 , PI/4 );
    public static Pose RLGrab1 = new Pose(-49, -42, PI/2);
    public static Pose RLForward1 = new Pose(-49, -40, PI/2);
    public static Pose RLGrab2 = new Pose(-59, -42, PI/2);
    public static Pose RLForward2 = new Pose(-59, -40, PI/2);
    public static Pose RLGrab3 = new Pose(-50, -26, PI);
    public static Pose RLGrab3Mid = new Pose(-40, -30, PI);
    public static Pose RLForward3 = new Pose(-54, -26, PI);
    public static Pose RLPark = new Pose(-13 - HalfChassisLength, -3.5 - HalfChassisWidth, PI);
    public static Pose RLParkMid = new Pose(-60, -12, PI/2);

    public static Pose BLStartBar = new Pose(0 ,0, 0);
    public static Pose BLBar = new Pose(0, 36, 0);
    public static Pose BLStartBucket = new Pose( 46 - HalfChassisWidth,70.125 - HalfChassisLength, -PI/2);
    public static Pose BLScore = new Pose(61 - 5.75,61 - 5.75 , -3*PI/4 );
    public static Pose BLGrab1 = new Pose(49, 42, -PI/2);
    public static Pose BLForward1 = new Pose(49, 40, -PI/2);
    public static Pose BLGrab2 = new Pose(59, 42, -PI/2);
    public static Pose BLForward2 = new Pose(59, 40, -PI/2);
    public static Pose BLGrab3 = new Pose(50, 26, 0);
    public static Pose BLGrab3Mid = new Pose(40, 30, 0);
    public static Pose BLForward3 = new Pose(54, 26, 0);
    public static Pose BLPark = new Pose(13 + HalfChassisLength, 3.5 + HalfChassisWidth, 0);
    public static Pose BLParkMid = new Pose(60, 12, -PI/2);





}
