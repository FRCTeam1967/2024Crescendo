// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private double velocity;
  
  /**
   * Creates a new RunShooter
   * @param shooter - Shooter object
   * @param speaker - true if shooting into speaker, false if shooting into amp
   */
  public RunShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
    velocity = Constants.Shooter.SPEAKER_VELOCITY;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.runShooter(velocity);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}