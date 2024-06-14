package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveTune extends Command {

    private final Swerve swerve;

    private DoubleSubscriber speedSrc;
    private DoubleSubscriber angleSrc;
    private BooleanSubscriber optSrc;

   public SwerveTune (Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

        NetworkTableInstance.getDefault().getDoubleTopic("/Swerve/TuneSpeed").publish().set(0);
        NetworkTableInstance.getDefault().getDoubleTopic("/Swerve/TuneAngle").publish().set(0);
        NetworkTableInstance.getDefault().getBooleanTopic("/Swerve/Optimize").publish().set(true);
        speedSrc = NetworkTableInstance.getDefault()
            .getDoubleTopic("/Swerve/TuneSpeed").subscribe(0.0);
        angleSrc = NetworkTableInstance.getDefault()
            .getDoubleTopic("/Swerve/TuneAngle").subscribe(0.0);
        optSrc = NetworkTableInstance.getDefault()
            .getBooleanTopic("/Swerve/Optimize").subscribe(true);
    }

    
    @Override
    public void execute() {
        double speedMPS = speedSrc.get();
        double angleDeg = angleSrc.get();
        boolean opt = optSrc.get();

        Rotation2d angleR2D = Rotation2d.fromDegrees(angleDeg);

        SwerveModuleState[] moduleState = new SwerveModuleState[4];
        for(int i = 0 ; i < 4; i++)
            moduleState[i] = new SwerveModuleState(speedMPS,angleR2D);
        swerve.setModuleStates(moduleState,opt);
    }


    @Override
    public void end (boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
