// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunFeeder extends Command {
  private Feeder feeder;
  private double speed;
  
  /**
   * Creates a new RunFeeder
   * @param feeder - Feeder object
   * @param speed
   */
  public RunFeeder(Feeder feeder, double speed) {
    this.feeder = feeder;
    this.speed = speed;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feeder.feedFeeder(speed);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}