// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class MovePivot extends Command {
  private Pivot pivot;
  private double desiredRev;
  /** Creates a new PivotMove. */
  public MovePivot(Pivot pivot, double rev) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot; 
    desiredRev = rev;
    addRequirements(this.pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.moveTo(desiredRev);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (desiredRev==Constants.Pivot.INTAKE_SAFE){
      pivot.pivotHoming();
      pivot.setpoint.velocity = 0;
      pivot.setpoint.position = pivot.getAbsPos();
      pivot.goal.velocity = 0;
      pivot.goal.position = pivot.getAbsPos();
    }*/
    return pivot.isReached();
  }
}
