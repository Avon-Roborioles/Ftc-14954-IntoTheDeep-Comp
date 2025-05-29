package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "frontLeft";
        FollowerConstants.leftRearMotorName = "backLeft";
        FollowerConstants.rightFrontMotorName = "frontRight";
        FollowerConstants.rightRearMotorName = "backRight";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.useBrakeModeInTeleOp = true;

        FollowerConstants.mass = 14.9;

        FollowerConstants.xMovement = 62.8461;
        FollowerConstants.yMovement = 48.1741;

        FollowerConstants.forwardZeroPowerAcceleration = -31.0466;
        FollowerConstants.lateralZeroPowerAcceleration = -66.0001;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.25,0.001,0.04,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.005,0,0.000025,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.translationalPIDFFeedForward = 0.015;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(0.725,0.001,0.00075,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.headingPIDFFeedForward = 0.025;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.00002,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.02,0.0000001,0.00015,0.6,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.drivePIDFFeedForward = 0.009;
        FollowerConstants.drivePIDFSwitch = 20;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
        FollowerConstants.maxPower = 0.9;
        FollowerConstants.motorCachingThreshold = 0.005;
    }
}
