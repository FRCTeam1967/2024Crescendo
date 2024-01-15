// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis; 
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class DriveStraight extends Command {
  /** Creates a new DriveStraight. */
  private double leftValue, rightValue;
  
private Chassis m_chassis;  
  public DriveStraight(double leftJoystickValue, double rightJoystickValue, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    leftValue = leftJoystickValue;
    rightValue = rightJoystickValue;
    m_chassis = chassis; 
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.driveStraight( 
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
