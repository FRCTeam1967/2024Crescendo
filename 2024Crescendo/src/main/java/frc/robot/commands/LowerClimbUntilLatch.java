// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class LowerClimbUntilLatch extends Command {
  private Climb climb;
  private DigitalInput sensor;

  public LowerClimbUntilLatch(Climb _climb, DigitalInput _sensor) {
    climb = _climb;
    sensor = _sensor;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!sensor.get()) climb.lowerAt(Constants.Climb.AUTOMATIC_LOWER_SPEED);
    else climb.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sensor.get(); //when sensor returns true
  }
}
