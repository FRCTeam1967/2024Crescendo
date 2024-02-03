// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenShooter;

public class RunKrakenShooter extends Command {

  private KrakenShooter krakenShooter;
  private double topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed;
  
  //Creates a new RunKra
  public RunKrakenShooter(KrakenShooter krakenShooter, double topLeftSpeed, double topRightSpeed, double bottomLeftSpeed, double bottomRightSpeed) {
    this.krakenShooter = krakenShooter;
    this.topLeftSpeed = topLeftSpeed;
    this.topRightSpeed = topRightSpeed;
    this.bottomLeftSpeed = bottomLeftSpeed;
    this.bottomRightSpeed = bottomRightSpeed;
     // Use addRequirements() here to declare subsystem dependencies.
    ;addRequirements(this.krakenShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    krakenShooter.runKrakenShooter(topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed);
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
