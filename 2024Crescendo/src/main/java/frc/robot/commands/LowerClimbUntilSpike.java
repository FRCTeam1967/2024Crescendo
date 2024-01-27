// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArm;

public class LowerClimbUntilSpike extends Command {
  private TelescopingArm arm;
  private double current;

  /**
   * Creates objects for arm and current fields 
   * @param arm - TelescopingArm object
   * @param current - DoubleSupplier for updating current value
   */
  public LowerClimbUntilSpike(TelescopingArm _arm, DoubleSupplier _current) {
    arm = _arm;
    current = _current.getAsDouble();
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.moveWinch(() -> Constants.TelescopingArm.AUTOMATIC_LOWER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  @Override
  public boolean isFinished() {
    return current >= Constants.TelescopingArm.SPIKE_CURRENT;
  }
}
