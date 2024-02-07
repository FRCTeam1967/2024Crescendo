// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Climb extends Subsystem {
  public void homeAtTop();
  public void moveTo(double pos, boolean holdingRobot);
  public void moveAt(DoubleSupplier speed);
  public boolean isReached(double desiredPos);
  public void switchMode();
  public void stop();
  public void maintainPos();
  public void configDashboard(ShuffleboardTab tab);
}
