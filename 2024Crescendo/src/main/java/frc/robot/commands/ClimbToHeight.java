// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.Constants;

public class ClimbToHeight extends Command {
  private Climb climb;
  private double height;

  /** 
   * Creates a new ClimbToHeight object
   * @param height - rotations to go to certain height
   * @param climb - Climb object
   */
  public ClimbToHeight(double _height, Climb _climb) {
    climb = _climb;
    height = _height;

    //configure PID based on if holding robot height or not
    if (height == Constants.Climb.LATCH_HEIGHT) climb.configurePID(true);
    else climb.configurePID(false);
    
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.moveTo(height);
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
