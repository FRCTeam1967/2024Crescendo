package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends Command {
  private final Swerve swerve;
  private DoubleSupplier xSupplier, ySupplier, rotationSupplier;
  private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

  public SwerveDrive (Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(swerve);
  }

  private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling) {
    input = Math.pow(input, 3);
    input = Math.abs(input) > deadband ? input : 0;
    input *= speedScaling;

    return input;
  }

  @Override
  public void execute() {
    double deadband = Constants.ExperimentalFeatures.applyDriverDeadband ? Constants.Xbox.DRIVER_DEADBAND : 0.0;
    double xSpeed = cleanAndScaleInput(deadband, xSupplier.getAsDouble(), xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
    double ySpeed = cleanAndScaleInput(deadband, ySupplier.getAsDouble(), yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
    double rotationSpeed = cleanAndScaleInput(deadband, rotationSupplier.getAsDouble(), rotationLimiter, Constants.Swerve.SWERVE_ROTATION_MAX_SPEED_IN_RAD);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, swerve.getRotation2d());
    SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleState);   
  }

  @Override
  public void end (boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static double signedSquare(double a) { //TODO: where are we using this?
    if (a < 0) return -(a * a);
    else return a * a;
  }
}
