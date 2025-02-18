package org.firstinspires.ftc.teamcode.commands.AutonomusCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.AutoDriveSubsystem;

public class AutoDriveCommand extends CommandBase {
    private AutoDriveSubsystem autoDriveSubsystem;
    private Telemetry telemetry;
    public AutoDriveCommand(AutoDriveSubsystem autoDriveSubsystem, Telemetry telemetry){
        this.autoDriveSubsystem = autoDriveSubsystem;
        this.telemetry = telemetry;
        addRequirements(autoDriveSubsystem);
    }

    @Override
    public void execute(){
        autoDriveSubsystem.update();
        autoDriveSubsystem.telemetryDebug(telemetry);
    }
    @Override
    public boolean isFinished(){
        return !autoDriveSubsystem.isBusy();
    }
    public void setPath(Path path, boolean holdEnd){
        autoDriveSubsystem.followPath(path, holdEnd);
    }
    public void setPathChain(PathChain pathChain, boolean holdEnd){
        autoDriveSubsystem.followPath(pathChain, holdEnd);
    }
}
