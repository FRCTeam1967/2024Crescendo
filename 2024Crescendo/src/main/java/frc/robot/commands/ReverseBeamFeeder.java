// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ReverseBeamFeeder extends Command {
  private Feeder feeder;
  
  /**
   * Creates a new RunFeeder
   * @param feeder - Feeder object
   */
  public ReverseBeamFeeder(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    feeder.feedFeeder(Constants.Feeder.REVERSE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return !feeder.isBroken();
  }
}
