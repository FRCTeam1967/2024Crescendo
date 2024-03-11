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
  private double topVelocity, topAcceleration,  bottomVelocity, bottomAcceleration;
  private Timer timer;
  
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
  public void initialize() {
    timer = new Timer();
    timer.start();
  }

  @Override
  public void execute() {
    shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
    // MDS: P2: A few things here. First, since you can (but don't currently) have top
    // and bottom velcoities that are different, taking the average is weird. Either commit
    // to always having the same, or make take the top and bottom averages separately. 
    // Second, you have this hard coded 90 here that doesn't change if you sent the velocities
    // to 50 or 180. It should probably be a percentage of the target the velocity(-ies).
    // Third, if we're in the middle of shooting, and the speeds drop below 90 as the note goes
    // through, we don't want to stop running the feeder. We should probably have a property
    // like reachedShootingSpeed that starts out False in initialize(), and once the average 
    // velocity gets to the point we want, set it to True. Then change the condition here to:
    //   if (reachedShooterSpeed || shooter.getAverageVelocity() >= 0.9 * speakerVelocity) {
    //       reachedShooterSpeed = true;
    //       shooter.runShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
    //       feeder.feedFeeder(Constants.Feeder.FEED_SPEED);
    //   }
    if (shooter.getAverageVelocity() >= 90){
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