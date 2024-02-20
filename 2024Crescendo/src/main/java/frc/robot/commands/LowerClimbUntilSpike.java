// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class LowerClimbUntilSpike extends Command {
  private Climb climb;
  private double current;

  /**
   * Creates a new LowerClimbUntilSpike object
   * @param pdhPort - PDH port for that motor
   * @param pdh - Power Distribution object
   * @param climb - Climb object
   */
  public LowerClimbUntilSpike(int pdhPort, PowerDistribution pdh, Climb _climb) {
    climb = _climb;
    current = pdh.getCurrent(pdhPort);
    addRequirements(climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climb.lowerAt(Constants.Climb.AUTOMATIC_LOWER_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }
  
  @Override
  public boolean isFinished() {
    return (current >= Constants.Climb.SPIKE_CURRENT) || //latched!
    (climb.getMotorPosition() >= Constants.Climb.SAFE_ROTATIONS && current <= Constants.Climb.HANG_CURRENT); //reached safe position but didn't hang on chain
  }
}
