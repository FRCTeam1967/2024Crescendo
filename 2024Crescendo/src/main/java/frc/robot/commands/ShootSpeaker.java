// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootSpeaker extends Command {
  private Shooter shooter;
  private Feeder feeder;
  private double topVelocity, topAcceleration, bottomVelocity, bottomAcceleration;
  private boolean reachedShooterSpeed = false;
  
  /**
   * Creates a new ShootSpeaker
   * @param shooter - Shooter object
   * @param feeder - Feeder object
   */
  public ShootSpeaker(Shooter shooter, Feeder feeder) {
    this.shooter = shooter;
    this.feeder = feeder;
    addRequirements(this.shooter, this.feeder);

    topVelocity = Constants.Shooter.SPEAKER_TOP_VELOCITY;
    topAcceleration = Constants.Shooter.SPEAKER_TOP_ACCELERATION;
    bottomVelocity = Constants.Shooter.SPEAKER_BOTTOM_VELOCITY;
    bottomAcceleration = Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
    if (reachedShooterSpeed || shooter.getAverageTopVelocity() >= Constants.Shooter.THRESHOLD_SPEED*0.9) {
      reachedShooterSpeed = true;
      shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
      feeder.feedFeeder(Constants.Feeder.FEED_SPEED);
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