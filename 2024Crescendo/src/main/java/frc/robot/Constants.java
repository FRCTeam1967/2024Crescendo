// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {
  public static class Xbox { 
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class Shooter {
    public static final int TOP_MOTOR_ID = 20, BOTTOM_MOTOR_ID = 22; //change
    
    public static final double SPEAKER_VELOCITY = 90.0;
    public static final double THRESHOLD_SPEED = 80;
  }
}