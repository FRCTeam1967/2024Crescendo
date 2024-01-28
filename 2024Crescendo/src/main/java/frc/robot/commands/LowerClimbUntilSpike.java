// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class LowerClimbUntilSpike extends Command {
  private Climb climb;
  private double current;

  /**
   * Creates objects for climb and current fields 
   * @param climb - Climb object
   * @param current - DoubleSupplier for updating current value
   */
  public LowerClimbUntilSpike(Climb _climb, DoubleSupplier _current) {
    climb = _climb;
    current = _current.getAsDouble();
    addRequirements(climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climb.moveWinch(() -> Constants.Climb.AUTOMATIC_LOWER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  @Override
  public boolean isFinished() {
    return current >= Constants.Climb.SPIKE_CURRENT;
  }
}
