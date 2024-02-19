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

public class WallSnapDrive extends Command {
  private double lastHeading;
  private final Swerve swerve;
  private DoubleSupplier xSupplier, ySupplier, headingSupplier;

  private final ProfiledPIDController angleController = new ProfiledPIDController(0.1, 0, 0.0, Constants.Swerve.SWERVE_ROTATION_PID_CONSTRAINTS);

  public WallSnapDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier headingSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.headingSupplier = headingSupplier;

    angleController.setTolerance(Constants.Swerve.SWERVE_ROTATION_TOLERANCE);
    angleController.enableContinuousInput(0, 360);
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
    double ySpeed = cleanAndScaleInput(0.00, ySupplier.getAsDouble(), Constants.Swerve.SWERVE_MAX_SPEED);

    double desiredHeading = headingSupplier.getAsDouble() != -1 ? headingSupplier.getAsDouble() : lastHeading;
    double rotSpeed = angleController.calculate(currentHeading(), desiredHeading);

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerve.getRotation2d());
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
