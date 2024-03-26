// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ManualClimb extends Command {
  private Climb climb;
  private DoubleSupplier speed;
  
  public ManualClimb(DoubleSupplier _speed, Climb _climb) {
    climb = _climb;
    speed = _speed;
    addRequirements(climb);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    if(!climb.isLocked() && !climb.getSensorValue() && climb.getEncoderCount() > Constants.Climb.TOP_ROTATIONS){
      climb.runMotor(speed);
    } else if(!climb.isLocked() && climb.getSensorValue() && speed.getAsDouble() < 0){
      climb.runMotor(speed);
    } else {
      climb.stop();
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
