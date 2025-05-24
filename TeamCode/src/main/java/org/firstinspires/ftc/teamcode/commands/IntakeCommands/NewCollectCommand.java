package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.concurrent.TimeUnit;

public class NewCollectCommand extends CommandBase {
    private NewIntakeSubsystem subsystem;
    private WristSubsystem wrist;
    private Telemetry telemetry;
    private boolean first=true;
    private boolean stalled = false;
    private boolean RedAlliance;

    Timing.Timer stall = new Timing.Timer(250, TimeUnit.MILLISECONDS);

    public NewCollectCommand(NewIntakeSubsystem subsystem, WristSubsystem wrist, Telemetry telemetry, boolean RedAlliance) {
        this.subsystem = subsystem;
        this.wrist = wrist;
        this.telemetry = telemetry;
        this.RedAlliance = RedAlliance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    if (RedAlliance) {
        subsystem.redAllianceLight();
    }else {
        subsystem.blueAllianceLight();
    }
    }

    @Override
    public void execute() {
        subsystem.getTelemetry(telemetry);
        if (!stalled){
            if (!subsystem.isStalled()){
                if (subsystem.hasSample()){
                    subsystem.stopMotor();
                    if (RedAlliance) {
                        if(!subsystem.isSampleRedOrYellow()){
                            subsystem.runMotor();
                        }
                    }else {
                        if(!subsystem.isSampleBlueOrYellow()){
                            subsystem.runMotor();
                        }
                    }
                }else {
                    subsystem.runMotor();
                    first = true;
                }

            }else {
                if (first){
                    stall.start();
                    subsystem.stopMotor();
                    first=false;
                    stalled = true;
                }

                subsystem.rejectMotor();

            }
        }else {
            if (stall.done()){
                subsystem.stopMotor();
                stalled = false;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopMotor();

    }

    @Override
    public boolean isFinished() {
        if (RedAlliance) {
            return subsystem.isSampleRedOrYellow();
        }else {
            return subsystem.isSampleBlueOrYellow();
        }
    }

}
