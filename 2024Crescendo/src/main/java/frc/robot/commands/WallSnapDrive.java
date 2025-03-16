package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class WallSnapDrive extends Command {
  private double lastHeading;
  private final Swerve swerve;
  private DoubleSupplier xSupplier, ySupplier, headingSupplier;

  private final ProfiledPIDController angleController =
    new ProfiledPIDController(0.1, 0, 0.0, Constants.Swerve.SWERVE_ROTATION_PID_CONSTRAINTS);

  public WallSnapDrive(Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier headingSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.headingSupplier = headingSupplier;

    angleController.setTolerance(Constants.Swerve.SWERVE_ROTATION_TOLERANCE);
    angleController.enableContinuousInput(0, 360);
    addRequirements(swerve);
  }

  private double currentHeading() {
    // return swerve.getRotation2d().getDegrees();
    return swerve.getHeading().getDegrees();
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
    double xSpeed, ySpeed, desiredHeading, rotSpeed;

    if(RobotContainer.redAlliance && headingSupplier.getAsDouble()==270){
        xSpeed = cleanAndScaleInput(0.00, xSupplier.getAsDouble(), Constants.Swerve.SWERVE_MAX_SPEED);
        ySpeed = cleanAndScaleInput(0.00, ySupplier.getAsDouble(), Constants.Swerve.SWERVE_MAX_SPEED);

        desiredHeading = headingSupplier.getAsDouble() != -1 ? (headingSupplier.getAsDouble()-180) : lastHeading;
        rotSpeed = angleController.calculate(currentHeading(), desiredHeading);
    }else{
        xSpeed = cleanAndScaleInput(0.00, xSupplier.getAsDouble(), Constants.Swerve.SWERVE_MAX_SPEED);
        ySpeed = cleanAndScaleInput(0.00, ySupplier.getAsDouble(), Constants.Swerve.SWERVE_MAX_SPEED);

        desiredHeading = headingSupplier.getAsDouble() != -1 ? headingSupplier.getAsDouble() : lastHeading;
        rotSpeed = angleController.calculate(currentHeading(), desiredHeading);
    }

    swerve.driveFieldRelative(xSpeed, ySpeed, rotSpeed);
  }

  public void end (boolean interrupted) {
    swerve.stopModules();
  }

  public boolean isFinished() {
    return false;
  }
}
