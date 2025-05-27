package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Storage;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.AfterAutoReset;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.CancelCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.IntakeToReadyForBottomScore;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.FixExtensionCommand;
import org.firstinspires.ftc.teamcode.commands.HangCommands.HangHoldCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.Score;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.HangCommands.HangJoystickCommand;
import org.firstinspires.ftc.teamcode.commands.CommandGroups.IntakeToReadyForTopScore;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.Reject;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.ClipTopSpecimen;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomResetCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopBarCommand;

import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.TeleDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TelePathDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleSlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "BlueTeleOp")
public class CompTeleOpBlue extends Storage {
    private Motor  liftMotor;
    private GamepadEx driverOp, operatorOp;
    private TouchSensor touch1, touch2;
    private ServoImplEx extendservo;
    PwmControl.PwmRange servoRange = new PwmControl.PwmRange(799, 1500);
    private Follower follower;
    private AutoDriveSubsystem autoDriveSubsystem;
    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private NewIntakeSubsystem intake;
    private WristSubsystem wrist;
    private HangSubsystem hang;
    private ClawSubsystem claw;
    private Path scorePath;
    @Override
    public void initialize() {

        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);
        claw = new ClawSubsystem (hardwareMap.get(Servo.class, "claw"));
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_312);
        touch1 = hardwareMap.get(TouchSensor.class, "liftDown");
        touch2 = hardwareMap.get(TouchSensor.class, "extensionIn");

        extendservo = hardwareMap.get(ServoImplEx.class, "extension");
        extendservo.setPwmRange(servoRange);
        extend = new ExtendSubsystem(extendservo, touch2 );
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"), hardwareMap.get(TouchSensor.class, "swingArmDown"));
        liftSubsystem = new LiftSubsystem(liftMotor, touch1);
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        hang = new HangSubsystem(new Motor(hardwareMap, "climb"));
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(Storage.memory.lastPose);
        follower.setPose(Storage.memory.lastPose);
        follower.startTeleopDrive();





        autoDriveSubsystem = new AutoDriveSubsystem(follower, telemetry, Storage.memory.lastPose);
        initPoseSelect(driverOp);

        autoDriveSubsystem.setDefaultCommand(new TeleDriveCommand(autoDriveSubsystem, telemetry, driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX, true));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenHeld(new TeleSlowDriveCommand(autoDriveSubsystem, telemetry, driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX, true))
                .whenInactive(new TeleDriveCommand(autoDriveSubsystem, telemetry, driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX, true));

        intake = new NewIntakeSubsystem(hardwareMap.get(DcMotorEx.class, "Intake"), hardwareMap.get(ColorSensor.class, "intakeColor"), hardwareMap.get(ColorSensor.class, "RampColor"), hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"), hardwareMap.get(ServoImplEx.class, "allianceColor"), false);
        hang.setDefaultCommand(new HangJoystickCommand(hang, operatorOp::getLeftY));



        /*
        Command Bindings
         */
        // Lift Commands
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .toggleWhenPressed(new HangHoldCommand(hang));

        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {follower.setPose(new Pose(0, 0, PI));}));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new TelePathDriveCommand(autoDriveSubsystem, follower.getPose()));

        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(new FixExtensionCommand(extend));

        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ClipTopSpecimen(liftSubsystem, 2000));

        operatorOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new LiftBottomResetCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new LiftBottomCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new LiftTopBarCommand(liftSubsystem));
        //Wrist Commands
        operatorOp.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new LowerWrist(wrist), new RaiseWrist(wrist));
        //Extend (Bumpers)
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new RaiseWrist(wrist))
                .whenPressed(new ExtendCommand(extend));
        operatorOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new RaiseWrist(wrist))
                .whenPressed(new RetractCommand(extend));
        //Other Commands
        operatorOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new CancelCommand(intake, liftSubsystem));
        // Score Command
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new Score(swingArmSubsystem, liftSubsystem, claw, intake));
        // Intake Commands
        operatorOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new IntakeToReadyForTopScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw));

        operatorOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new IntakeToReadyForBottomScore(intake, wrist, extend, swingArmSubsystem, liftSubsystem, claw));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .toggleWhenPressed(new SwingArmUpCommand(swingArmSubsystem), new SwingArmDownCommand(swingArmSubsystem));


        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(new WaitCommand(10), new LiftBottomResetCommand(liftSubsystem), new AfterAutoReset(liftSubsystem, swingArmSubsystem), new RaiseWrist(wrist), new RetractCommand(extend)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (opModeInInit() && !isStopRequested()){
            runPoseSelect(telemetry);
            follower.setPose(Storage.memory.lastPose);
            follower.drawOnDashBoard();
        }
        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
//            scorePath = new Path(new BezierCurve(new Point(follower.getPose()),  new Point(new Pose(-57 ,-54 , PI/4 ))));
            run();
        }
        reset();
    }


}
