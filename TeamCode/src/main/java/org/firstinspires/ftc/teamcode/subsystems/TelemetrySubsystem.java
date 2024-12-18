package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    private Telemetry telemetry;

    private BoxxySubsystem boxxySubsystem;
    private ExtendSubsystem extendSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LiftSubsystem liftSubsystem;
    private PassSubsystem passSubsystem;
    private PedroDriveSubsystem pedroDriveSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private WristSubsystem wristSubsystem;

    public TelemetrySubsystem (Telemetry telemetry, BoxxySubsystem boxxySubsystem, ExtendSubsystem extendSubsystem, IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem, PassSubsystem passSubsystem, PedroDriveSubsystem pedroDriveSubsystem, SwingArmSubsystem swingArmSubsystem, WristSubsystem wristSubsystem){
        this.telemetry = telemetry;
        this.boxxySubsystem = boxxySubsystem;
        this.extendSubsystem = extendSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.passSubsystem = passSubsystem;
        this.pedroDriveSubsystem = pedroDriveSubsystem;
        this.swingArmSubsystem = swingArmSubsystem;
        this.wristSubsystem = wristSubsystem;


    }
    public void getTelemetry(){
        clearTelemetry();
        boxxySubsystem.getTelemetry(telemetry);
        extendSubsystem.getTelemetry(telemetry);
        intakeSubsystem.getTelemetry(telemetry);
        liftSubsystem.getTelemetry(telemetry);
        passSubsystem.getTelemetry(telemetry);
//        pedroDriveSubsystem.telemetryDebug(telemetry);
        swingArmSubsystem.getTelemetry(telemetry);
        wristSubsystem.getTelemetry(telemetry);
        updateTelemetry();
    }
    public void updateTelemetry(){
        telemetry.update();
    }
    public void clearTelemetry(){
        telemetry.clearAll();
    }



}
