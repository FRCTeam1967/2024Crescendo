// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class GoToMaxHeight extends Command {
  private Climb m_climb;

  /** Creates a new GoToMaxHeight. */
  public GoToMaxHeight(Climb climb) {
    m_climb = climb;
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //create profile based on current state
    //disable "hold position" logic
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //follow the profile
    m_climb.moveWinch(() -> Constants.Climb.MAX_WINCH_SPEED, () -> Constants.Climb.MAX_WINCH_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //turn motor power to 0
    //reenable hold position
    m_climb.moveWinch(() -> 0.0, () -> 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climb.encoderCheck();
  }
}
