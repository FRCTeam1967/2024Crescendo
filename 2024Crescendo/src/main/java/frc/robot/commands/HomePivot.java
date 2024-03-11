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
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // MDS: P3: This seems weird to be in the isFinished() method. Generally, isFinished() shouldn't have side effects
    // (modify state). This seems like you'd want this all to be in initialize or execute, and that isFinished() would return
    // true always. Do we really need to wait for the profile if we just set the goal and state to be the same? 
    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getAbsPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getAbsPos();

    SmartDashboard.putString("Did we finish", "yes");

    return pivot.isReached();
  }
}