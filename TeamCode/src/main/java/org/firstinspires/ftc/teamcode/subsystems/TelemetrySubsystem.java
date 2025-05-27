package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    private Telemetry telemetry;

    private ExtendSubsystem extendSubsystem;
    private LiftSubsystem liftSubsystem;
    private PedroDriveSubsystem pedroDriveSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private WristSubsystem wristSubsystem;

    public TelemetrySubsystem (Telemetry telemetry, ExtendSubsystem extendSubsystem, LiftSubsystem liftSubsystem, PedroDriveSubsystem pedroDriveSubsystem, SwingArmSubsystem swingArmSubsystem, WristSubsystem wristSubsystem){
        this.telemetry = telemetry;
        this.extendSubsystem = extendSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.pedroDriveSubsystem = pedroDriveSubsystem;
        this.swingArmSubsystem = swingArmSubsystem;
        this.wristSubsystem = wristSubsystem;


    }
    public void getTelemetry(){
        clearTelemetry();
        extendSubsystem.getTelemetry(telemetry);
        liftSubsystem.getTelemetry(telemetry);
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
