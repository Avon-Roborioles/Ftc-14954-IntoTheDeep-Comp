package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightSubsystem extends SubsystemBase {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private LLResult result;
    private final int startingPipeline = 5;
    public LimelightSubsystem(Limelight3A limelight, Telemetry telemetry){
        this.limelight = limelight;
        limelight.pipelineSwitch(startingPipeline);
        limelight.start();
        this.telemetry = telemetry;
    }
    public LLResult readAprilTag(){
        getResult();
        result = limelight.getLatestResult();
        return result;

    }
    public LLResult lookForSpecimen(){
        getResult();
        result = limelight.getLatestResult();
        return result;

    }


    public void getLimelightTelemetryAprilTag(){
        readAprilTag();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("tags", result.getBotposeTagCount());
            }
        }
        telemetry.update();
    }
    public void getLimelightTelemetrySpecimen(Follower follower){
        Pose lastPose = follower.getPose();
        lookForSpecimen();
        double heading =lastPose.getHeading() - PI/2;
        double y = 60 + lastPose.getY();
        double x = lastPose.getX();
        double strafe = -Math.tan(Math.toRadians(result.getTx() + heading))*y + x;

        if (result != null) {
            telemetry.addData("BotPose", lastPose);
            telemetry.addData("heading", heading);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("strafe", strafe);

            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Ta", result.getTa());
            telemetry.addData("Results", result.getDetectorResults());
        }
        telemetry.update();
    }
    public double getStrafe(Follower follower){
        Pose lastPose = follower.getPose();
        lookForSpecimen();
        double heading =lastPose.getHeading() - PI/2;
        double y = 60 + lastPose.getY();
        double x = lastPose.getX();
        double strafe = -Math.tan(Math.toRadians(result.getTx()+ heading))*y + x;
        getLimelightTelemetrySpecimen(follower);
        return strafe;
    }
    public void getResult(){
        result = limelight.getLatestResult();
    }
    public void setPipeline(int pipeline){
        limelight.stop();
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }
    public double getYawAprilTag(){
        return readAprilTag().getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
    }
}