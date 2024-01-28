// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbToLowHeight extends Command {
  private Climb climb;

  /** 
   * Creates new ClimbToLowHeight object
   * @param climb - Climb object
   */
  public ClimbToLowHeight(Climb _climb) {
    climb = _climb;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.moveTo(Constants.Climb.LOW_WINCH_ROTATIONS);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  @Override
  public boolean isFinished() {
    return climb.isReached();
  }
}
