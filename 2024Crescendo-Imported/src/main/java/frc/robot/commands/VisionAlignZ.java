// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class VisionAlignZ extends Command {
  private final Swerve swerve;
  private final Vision vision;
  private SlewRateLimiter yLimiter;
  public VisionAlignZ(Swerve swerve, Vision vision) {
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
    if (vision.getAlliance()){
      double ySpeed = cleanAndScaleInput(0, -0.35, yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, ySpeed, 0.0, swerve.getRotation2d());
      SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      swerve.setModuleStates(moduleState);
    }else{
      double ySpeed = cleanAndScaleInput(0, 0.35, yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
      ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, ySpeed, 0.0, swerve.getRotation2d());
      SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      swerve.setModuleStates(moduleState);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.limelightToGoalInches < 19 //stops detecting the limelight around 18 inches
    ;
  }
}
