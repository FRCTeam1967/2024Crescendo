// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenShooter;

public class RunKrakenShooter extends Command {
  private KrakenShooter krakenShooter;
  private double topVelocity, topAcceleration,  bottomVelocity, bottomAcceleration;
  
  //Creates a new RunKrakenShooter
  public RunKrakenShooter(KrakenShooter krakenShooter, double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    this.krakenShooter = krakenShooter;
    this.topVelocity = topVelocity;
    this.topAcceleration = topAcceleration;
    this.bottomVelocity = bottomVelocity;
    this.bottomAcceleration = bottomAcceleration;
    addRequirements(this.krakenShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    krakenShooter.runKrakenShooter(topVelocity, topAcceleration, bottomVelocity, bottomAcceleration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    krakenShooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}