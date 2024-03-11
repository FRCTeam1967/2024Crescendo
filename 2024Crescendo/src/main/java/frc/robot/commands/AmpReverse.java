package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AmpReverse extends Command {
  private final Swerve swerve;
  private SlewRateLimiter yLimiter;
  private Timer timer;

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
    timer = new Timer();
    timer.start();
  }

  private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling) {
    input = Math.pow(input, 3);
    input = Math.abs(input) > deadband ? input : 0;
    input *= speedScaling;

    return input;
  }
    
  @Override
  public void execute() {
    double ySpeed = cleanAndScaleInput(0.00, Constants.Swerve.AMP_REVERSE, yLimiter, (Constants.Swerve.SWERVE_MAX_SPEED)/2);
    // MDS: P3: I think this was working, but I don't understand why. We're using field relative speeds, so the X axis is between the alliance
    // walls. But the amp is on the side wall, so wouldn't we want to be changing the y value, or be using 
    // robot-relative speeds (where X is forward) since we can assume (?) the driver drove into the amp shooter first? In the latter case,
    // that would just be creating a ChassisSpeeds object directly -- i.e.:
    //   ChassisSpeeds chassisSpeeds = new ChassisSpeeds(ySpeed, 0, 0);
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, 0, 0, swerve.getRotation2d());
    SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleState);
  }

  @Override
  public void end (boolean interrupted) {
    swerve.stopModules();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= 0.6;
  }
}
