// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class LowerClimbUntilLatch extends Command {
  private Climb climb;

  public LowerClimbUntilLatch(Climb _climb) {
    climb = _climb;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climb.lowerAt(Constants.Climb.AUTOMATIC_LOWER_SPEED); 
  }

  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  @Override
  public boolean isFinished() {
    return !climb.getSensorValue();
  }
}
