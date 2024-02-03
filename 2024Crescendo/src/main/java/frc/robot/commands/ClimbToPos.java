// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.Constants;

public class ClimbToPos extends Command {
  private Climb climb;
  private double pos;

  /** 
   * Creates a new ClimbToPos object
   * @param pos - rotations to go to desired position
   * @param climb - Climb object (either ClimbNEO or ClimbKraken)
   */
  public ClimbToPos(double _pos, Climb _climb) {
    climb = _climb;
    pos = _pos;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    if (pos == Constants.Climb.LATCH_ROTATIONS) climb.moveTo(pos, true);
    else climb.moveTo(pos, false);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  @Override
  public boolean isFinished() {
    return climb.isReached(pos);
  }
}
