// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class LowerClimbUntilSpike extends Command {
  private Climb climb;
  private PowerDistribution pdh;
  private int pdhPort;

  /**
   * Creates a new LowerClimbUntilSpike object
   * @param pdhPort - PDH port for that motor
   * @param pdh - Power Distribution object
   * @param climb - Climb object
   */
  public LowerClimbUntilSpike(int _pdhPort, PowerDistribution _pdh, Climb _climb) {
    climb = _climb;
    pdh = _pdh;
    pdhPort = _pdhPort;
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
    double current = pdh.getCurrent(pdhPort);
    return (current >= Constants.Climb.SPIKE_CURRENT); //latched
  }
}
