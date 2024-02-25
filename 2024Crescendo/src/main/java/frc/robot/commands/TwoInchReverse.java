package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TwoInchReverse extends Command {
    private double lastHeading;
    private double currentPosition;
    private double goalPosition;
    private final Swerve swerve;
    private DoubleSupplier xSupplier, ySupplier, headingSupplier;

    private final ProfiledPIDController angleController =
            new ProfiledPIDController(0.1, 0, 0.0, Constants.Swerve.SWERVE_ROTATION_PID_CONSTRAINTS);

    public TwoInchReverse(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier headingSupplier) {
        this.swerve = swerve;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.headingSupplier = headingSupplier;

        angleController.setTolerance(Constants.Swerve.SWERVE_ROTATION_TOLERANCE);
        angleController.enableContinuousInput(0, 360);

        currentPosition = 0;
        goalPosition = 0-(2/(4*Math.PI)) * Constants.Swerve.DRIVE_GEAR_RATIO; //revs backwards
    }

    private double currentHeading() {
        return swerve.getRotation2d().getDegrees();
    }

    public void initialize() {
        angleController.reset(currentHeading(), 0);
        lastHeading = currentHeading();
    }

    private double cleanAndScaleInput(double deadband, double input, double speedScaling) {
        input = Math.pow(input, 3);
        input = Math.abs(input) > deadband ? input : 0;
        input *= speedScaling;

        return input;
    }

    public void execute() {
        double xSpeed = cleanAndScaleInput(0.00, xSupplier.getAsDouble(), Constants.Swerve.SWERVE_MAX_SPEED);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0, 0, swerve.getRotation2d());
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, Constants.Swerve.SWERVE_MAX_SPEED);
        swerve.setModuleStates(moduleState);
    }


    public void end (boolean interrupted) {
        swerve.stopModules();
    }

    public boolean isFinished() {
        return false;
    }


}
