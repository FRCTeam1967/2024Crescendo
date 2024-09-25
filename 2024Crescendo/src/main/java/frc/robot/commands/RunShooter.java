// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private double velocity;
  // private double acceleration;
  private boolean reachedShooterSpeed = false;
  
  /**
   * Creates a new RunShooter
   * @param shooter - Shooter object
   * @param feeder - Feeder object
   */
  public RunShooter(Shooter shooter, double velocity) {
    this.shooter = shooter;
    this.velocity = velocity;
    addRequirements(this.shooter);
    
    //change constants
    // acceleration = Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() { 
    /*shooter.runShooter(velocity, 0); //change second to negative if inverted after testing
    if (reachedShooterSpeed || Constants.Shooter.SPEAKER_VELOCITY >= Constants.Shooter.THRESHOLD_SPEED*0.9) {
      reachedShooterSpeed = true;
      shooter.runShooter(velocity, velocity);
    }*/

    shooter.runShooter(velocity, velocity);

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