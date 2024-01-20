// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    SmartDashboard.putString("Reached?", "yes");
    pivot.shouldHold = false;
    pivot.shouldFollow = true;
    pivot.moveTo(desiredRev);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Ended?", "yes");
    pivot.shouldFollow = false;
    pivot.shouldHold = true;
    pivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pivot.isReached();
  }
}
