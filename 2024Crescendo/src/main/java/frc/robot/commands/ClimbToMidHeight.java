// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopingArm;

public class ClimbToMidHeight extends Command {
  private TelescopingArm arm;

  /** 
   * Creates new ClimbToMidHeight object
   * @param arm - TelescopingArm object
   */
  public ClimbToMidHeight(TelescopingArm _arm) {
    arm = _arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.moveTo(Constants.TelescopingArm.MID_WINCH_ROTATIONS);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  @Override
  public boolean isFinished() {
    return arm.isReached();
  }
}
