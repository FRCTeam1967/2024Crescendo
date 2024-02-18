// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private double topVelocity, topAcceleration,  bottomVelocity, bottomAcceleration;
  
  /**
   * Creates a new RunShooter
   * @param shooter - Shooter object
   * @param topVelocity
   * @param topAcceleration
   * @param bottomVelocity
   * @param bottomAcceleration
   */
  public RunShooter(Shooter shooter, double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    this.shooter = shooter;
    this.topVelocity = topVelocity;
    this.topAcceleration = topAcceleration;
    this.bottomVelocity = bottomVelocity;
    this.bottomAcceleration = bottomAcceleration;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}