package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
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
    double xSpeed = cleanAndScaleInput(0.00, xSupplier.getAsDouble(), xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
    double ySpeed = cleanAndScaleInput(0.00, ySupplier.getAsDouble(), yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
    double rotationSpeed = cleanAndScaleInput(0.00, rotationSupplier.getAsDouble(), rotationLimiter, Constants.Swerve.SWERVE_ROTATION_MAX_SPEED_IN_RAD);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, swerve.getRotation2d());
    SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleState);
    
    
    
    //megaTag use to estimate
    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        swerve.m_poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", swerve.m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(swerve.m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        swerve.m_poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }





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
