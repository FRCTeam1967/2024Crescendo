// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class RunFeederShooter extends Command {
  private Shooter shooter;
  private Feeder feeder;
  private double topVelocity, topAcceleration,  bottomVelocity, bottomAcceleration;
  private Timer timer;
  
  /**
   * Creates a new RunShooter
   * @param shooter - Shooter object
   * @param topVelocity
   * @param topAcceleration
   * @param bottomVelocity
   * @param bottomAcceleration
   */
  public RunFeederShooter(Shooter shooter, Feeder feeder, double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    this.shooter = shooter;
    this.topVelocity = topVelocity;
    this.topAcceleration = topAcceleration;
    this.bottomVelocity = bottomVelocity;
    this.bottomAcceleration = bottomAcceleration;
    this.feeder = feeder;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute() {
    shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
    if (shooter.getVelocity() >= 65){
      shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
      feeder.feedFeeder(Constants.Feeder.FEED_SPEED, Constants.Feeder.FEED_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
    feeder.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}