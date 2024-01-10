// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class WristMove extends Command {
  private Wrist wrist;
  private double desiredAngle;
  /** Creates a new WristMove. */
  public WristMove(Wrist wrist, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist; 
    desiredAngle = angle;
    addRequirements(this.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.moveTo(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.isReached();
  }
}
