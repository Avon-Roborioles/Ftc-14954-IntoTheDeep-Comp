package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.maxPower= 1;
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

        FollowerConstants.xMovement = 66.12;
        FollowerConstants.yMovement = 50.97;

        FollowerConstants.forwardZeroPowerAcceleration = -34.57;
        FollowerConstants.lateralZeroPowerAcceleration = -72.17;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.5,0,0.04,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.3,0,0.02,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.translationalPIDFFeedForward = 0.015;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2.5,0,0.125,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.009,0,0.0005,0.05,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0.0001,0.01,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.drivePIDFFeedForward = 0.01;

        FollowerConstants.zeroPowerAccelerationMultiplier = 5;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.motorCachingThreshold = 0.01;
        FollowerConstants.automaticHoldEnd = true;

        FollowerConstants.useVoltageCompensationInAuto= true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.nominalVoltage = 12;
        FollowerConstants.cacheInvalidateSeconds = 0.5;

    }
}
