// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootAmp extends Command {
  private Shooter shooter;
  private double topVelocity, topAcceleration,  bottomVelocity, bottomAcceleration;
  
  /**
   * Creates a new ShootAmp
   * @param shooter - Shooter object
   */
  public ShootAmp(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);

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

