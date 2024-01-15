// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.Chassis;

public class DriveSlow extends Command {
  // add fields
  private double leftValue, rightValue;
  private Chassis m_chassis;

  public DriveSlow(double leftJoystickValue, double rightJoystickValue, Chassis chassis) {
    leftValue = leftJoystickValue;
    rightValue = rightJoystickValue;
    m_chassis = chassis;
    addRequirements (m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.driveSlow( 
      () -> MathUtil.applyDeadband(leftValue, Constants.Chassis.JOYSTICK_DEADBAND),
      () -> MathUtil.applyDeadband(rightValue, Constants.Chassis.JOYSTICK_DEADBAND));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
