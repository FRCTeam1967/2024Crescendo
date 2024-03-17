package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AmpReverse extends Command {
  private final Swerve swerve;
  private SlewRateLimiter yLimiter;
  private Timer timer;
  private double initialEncPosition;

  /**
   * Creates a new AmpReverse
   * @param swerve - Swerve object
   * @param ySupplier - DoubleSupplier for speed
   */
  public AmpReverse(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  public void initialize() {
    initialEncPosition = swerve.backLeft.getEncoderPosition();
  }

  private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling) {
    input = Math.pow(input, 3);
    input = Math.abs(input) > deadband ? input : 0;
    input *= speedScaling;

    return input;
  }
    
  @Override
  public void execute() {
    SmartDashboard.putNumber("Initial Position", initialEncPosition);

    double ySpeed = cleanAndScaleInput(0.00, -Constants.Swerve.AMP_REVERSE_JS_INPUT, yLimiter, (Constants.Swerve.SWERVE_MAX_SPEED)/2);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, ySpeed, 0, swerve.getRotation2d());
    SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleState);
  }

  @Override
  public void end (boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    if ((swerve.backLeft.getEncoderPosition() >= initialEncPosition + Constants.Swerve.REVERSE_OFFSET/Constants.Swerve.METERS_TO_ENC_COUNT) || (swerve.backLeft.getEncoderPosition() <= initialEncPosition - Constants.Swerve.REVERSE_OFFSET/Constants.Swerve.METERS_TO_ENC_COUNT)){
      return true;
    }
    return false;
  }
}