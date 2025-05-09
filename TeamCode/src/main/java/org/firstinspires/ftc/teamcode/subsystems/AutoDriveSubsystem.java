package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Drawing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Storage;


public class AutoDriveSubsystem extends SubsystemBase {
    private Follower follower;
    private Telemetry telemetry;

    public AutoDriveSubsystem(Follower follower, Telemetry telemetry, Pose startPose){
        this.follower = follower;
        this.telemetry = telemetry;
        setPose(startPose);
    }

    public void followPath(Path path, boolean holdEnd){
        follower.followPath(path, holdEnd);
    }
    public void followPath(PathChain pathChain, boolean holdEnd){
        follower.followPath(pathChain, holdEnd);
    }
    public void setMaxPower(double maxPower){
        follower.setMaxPower(maxPower);
    }
    public void setPose(Pose pose){
        follower.setPose(pose);
    }
    public void setStartingPose(Pose pose){
        follower.setStartingPose(pose);
    }
    public void holdPoint(BezierPoint point, double heading){
        follower.holdPoint(point, heading);
    }
    public void update(){
        follower.update();
        Storage.memory.lastPose = follower.getPose();
        follower.drawOnDashBoard();
//        follower.telemetryDebug(telemetry);
    }
    public boolean isBusy(){
        return follower.isBusy();
    }
    public void breakFollowing(){
        follower.breakFollowing();
    }
    public PathBuilder pathBuilder(){
        return follower.pathBuilder();
    }
    public void telemetryDebug(Telemetry telemetry){
        follower.telemetryDebug(telemetry);
    }
    public Path getCurrentPath(){
        return follower.getCurrentPath();
    }
    public boolean atParametricEnd(){
        return follower.atParametricEnd();
    }
    public void setCurrentPoseWithOffset(Pose pose){
        follower.setCurrentPoseWithOffset(pose);
    }
    public void setXOffset(double xOffset){
        follower.setXOffset(xOffset);
    }
    public void setYOffset(double yOffset){
        follower.setYOffset(yOffset);
    }
    public void setHeadingOffset(double headingOffset){
        follower.setHeadingOffset(headingOffset);
    }
    public Pose getPose(){
        return follower.getPose();
    }
    public void resetOffset(){
        follower.resetOffset();
    }
    public void startTeleopDrive(){
        follower.startTeleopDrive();
    }
    public void setTeleOpMovementVectors(double forwardSpeed, double strafeSpeed, double heading, boolean robotCentric){
        follower.setTeleOpMovementVectors(-strafeSpeed,forwardSpeed
                 , heading, false);
        follower.updatePose();
    }
    public void holdPosition(){
        follower.holdPoint(new BezierPoint(new Point(getPose())), getPose().getHeading());
    }
}
