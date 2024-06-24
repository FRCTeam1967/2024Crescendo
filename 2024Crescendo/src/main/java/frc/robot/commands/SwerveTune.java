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

    private BooleanSubscriber frontLEnable;
    private BooleanSubscriber frontREnable;
    private BooleanSubscriber backLEnable;
    private BooleanSubscriber backREnable;

    public SwerveTune(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
        String prefix = "/SmartDashboard/"+getName()+"/";

        NetworkTableInstance.getDefault().getDoubleTopic(prefix+"TuneSpeed").publish().set(0);
        NetworkTableInstance.getDefault().getDoubleTopic(prefix+"TuneAngle").publish().set(0);
        NetworkTableInstance.getDefault().getBooleanTopic(prefix+"Optimize").publish().set(true);

        NetworkTableInstance.getDefault().getBooleanTopic(prefix+"FrontLeft").publish().set(true);
        NetworkTableInstance.getDefault().getBooleanTopic(prefix+"FrontRight").publish().set(true);
        NetworkTableInstance.getDefault().getBooleanTopic(prefix+"BackLeft").publish().set(true);
        NetworkTableInstance.getDefault().getBooleanTopic(prefix+"BackRight").publish().set(true);

        speedSrc = NetworkTableInstance.getDefault()
                .getDoubleTopic(prefix+"TuneSpeed").subscribe(0.0);
        angleSrc = NetworkTableInstance.getDefault()
                .getDoubleTopic(prefix+"TuneAngle").subscribe(0.0);
        optSrc = NetworkTableInstance.getDefault()
                .getBooleanTopic(prefix+"Optimize").subscribe(true);
        frontLEnable = NetworkTableInstance.getDefault()
                .getBooleanTopic(prefix+"FrontLeft").subscribe(true);
        frontREnable = NetworkTableInstance.getDefault()
                .getBooleanTopic(prefix+"FrontRight").subscribe(true);
        backLEnable = NetworkTableInstance.getDefault()
                .getBooleanTopic(prefix+"BackLeft").subscribe(true);
        backREnable = NetworkTableInstance.getDefault()
                .getBooleanTopic(prefix+"BackRight").subscribe(true);

    }

    @Override
    public void execute() {
        double speedMPS = speedSrc.get();
        double angleDeg = angleSrc.get();
        boolean opt = optSrc.get();

        boolean moduleEnable[];

        moduleEnable = new boolean[4];

        moduleEnable[0] = frontLEnable.get();
        moduleEnable[1] = frontREnable.get();
        moduleEnable[2] = backLEnable.get();
        moduleEnable[3] = backREnable.get();

        Rotation2d angleR2D = Rotation2d.fromDegrees(angleDeg);

        SwerveModuleState[] moduleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            if (moduleEnable[i])
                moduleState[i] = new SwerveModuleState(speedMPS, angleR2D);
            else
                moduleState[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        }
        swerve.setModuleStates(moduleState, opt);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
