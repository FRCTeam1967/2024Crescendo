// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class HomePivot extends Command {
  private Pivot pivot;
  /** Creates a new HomePivot. */
  public HomePivot(Pivot pivot) {
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.pivotHoming();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //pivot.moveTo(pivot.getAbsPos());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.isReached();
  }
}
