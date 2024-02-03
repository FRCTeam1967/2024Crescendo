// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbNEO;
import frc.robot.Constants;

public class ClimbToPos extends Command {
  private ClimbNEO climb;
  private double pos;

  /** 
   * Creates a new ClimbToPos object
   * @param pos - rotations to go to desired position
   * @param climb - Climb object
   */
  public ClimbToPos(double _pos, ClimbNEO _climb) {
    climb = _climb;
    pos = _pos;

    //configure PID based on if holding robot height or not
    if (pos == Constants.Climb.LATCH_ROTATIONS) climb.configPID(true);
    else climb.configPID(false);
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.moveTo(pos);
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
