// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class VisionAlign extends Command {
  private final Swerve swerve;
  private final Vision vision;
  private SlewRateLimiter xLimiter, yLimiter;
  public VisionAlign(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve, vision);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling){
    input = Math.pow(input, 3);
    input = Math.abs(input)> deadband ? input : 0;
    input *= speedScaling;

    return input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed;
    if (vision. getOffset() > 0){
      xSpeed = cleanAndScaleInput(0, 0.35, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
    }else{
      xSpeed = cleanAndScaleInput(0, -0.35, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
    }
    swerve.driveFieldRelative(xSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.getIsInRange();
  }
}
