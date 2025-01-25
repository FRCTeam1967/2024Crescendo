// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class HomePivot extends Command {
  private Pivot pivot;
  
  /**
   * Creates a new HomePivot.
   * @param pivot - Pivot object
   */
  public HomePivot(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivot.setRelToAbs();
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getAbsPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getAbsPos();
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putString("Did we finish", "yes");

    return pivot.isReached();
  }
}