// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class RaiseClimb extends Command {
  private Climb climb;
  private boolean isZero;

  public RaiseClimb(Climb _climb, boolean isZero) {
    climb = _climb;
    this.isZero = isZero;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //climb.moveTo(Constants.Climb.TOP_ROTATIONS, false);
    climb.lowerAt(Constants.Climb.LOWER_SPEED); 
  }

  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  @Override
  public boolean isFinished() {
    /*if (!isZero){
      return true;
    }else if (climb.getPosition() < -180){
      return true;
    }else{
      return false;
    }*/

    if (climb.getPosition() < -135){
      return true;
    }else{
      return false;
    }
  }
}