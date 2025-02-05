package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static java.lang.Math.PI;

import com.pedropathing.localization.Pose;

public class PoseList {
    

    // 46 6/8 - 1/2 robot width

    public static double HalfChassisWidth = 17.125/2;
    public static double HalfChassisLength = 8.1875;
    public static double HalfTilePuzzle = 0.75/2;

    public static Pose RLStartBar = new Pose(- HalfChassisWidth + 0.75 ,-69 + HalfChassisLength, -PI/2);
    public static Pose RLBar = new Pose(1 - HalfChassisWidth , -32, -PI/2);
    public static Pose RLBarMid = new Pose(1 - HalfChassisWidth , -38, -PI/2);
    public static Pose RLStartBucket = new Pose( -29.625 - HalfChassisWidth,-70.125 + HalfChassisLength, PI/2);
    public static Pose RLScore = new Pose(-57 ,-54 , PI/4 );
    public static Pose RLScorePreload = new Pose(-55 ,-55 , PI/4 );
    public static Pose RLGrab1 = new Pose(-49.5, -41.5, PI/2);
    public static Pose RLGrab1Mid = new Pose(-40, -50, PI/2);
    public static Pose RLForward1 = new Pose(-49.5, -38, PI/2);
    public static Pose RLGrab2 = new Pose(-60, -41.5, PI/2);
    public static Pose RLForward2 = new Pose(-60, -38, PI/2);
    public static Pose RLGrab3 = new Pose(-53, -34, 5* PI/6);
    public static Pose RLGrab3Mid = new Pose(-40, -36, 5* PI/6);
    public static Pose RLForward3 = new Pose(-56, -33, 5* PI/6);
    public static Pose RLPark = new Pose(-16.5 - HalfChassisLength, -2.5 - HalfChassisWidth, PI);
    public static Pose RLPrePark = new Pose(-25 - HalfChassisLength, -2.5 - HalfChassisWidth, PI);
    public static Pose RLParkMid = new Pose(-60, -11, PI/2);






    public static Pose BLStartBar = new Pose( HalfChassisWidth -0.75 ,68 - HalfChassisLength, PI/2);
    public static Pose BLBar = new Pose( HalfChassisWidth -0.75 , 31, PI/2);
    public static Pose BLBarMid = new Pose( HalfChassisWidth -0.75  , 38, PI/2);
    public static Pose BLStartBucket = new Pose( 29.625 + HalfChassisWidth,70.125 - HalfChassisLength, -PI/2);
    public static Pose BLScore = new Pose(57,54 , -3*PI/4 );
    public static Pose BLScorePreload = new Pose(55,55 , -3*PI/4 );
    public static Pose BLGrab1 = new Pose(49.5, 41, -PI/2);
    public static Pose BLGrab1Mid = new Pose(40, 50, -PI/2);
    public static Pose BLForward1 = new Pose(49.5, 38, -PI/2);
    public static Pose BLGrab2 = new Pose(59.5, 41, -PI/2);
    public static Pose BLForward2 = new Pose(59.5, 38, -PI/2);
    public static Pose BLGrab3 = new Pose(53, 34, -PI/6);
    public static Pose BLGrab3Mid = new Pose(40, 36, -PI/6);
    public static Pose BLForward3 = new Pose(56, 33, -PI/6);
    public static Pose BLPark = new Pose(16.5 + HalfChassisLength, 2.5 + HalfChassisWidth, 0);
    public static Pose BLPrePark = new Pose(25 + HalfChassisLength, 2.5 + HalfChassisWidth, 0);
    public static Pose BLParkMid = new Pose(60, 11, -PI/2);





}
