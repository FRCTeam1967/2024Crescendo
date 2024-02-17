// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class Climb {
    public static final int LEFT_MOTOR_ID = 30, RIGHT_MOTOR_ID = 31;
    public static final int LEFT_MOTOR_PDH_PORT = 17, RIGHT_MOTOR_PDH_PORT = 0;
    
    public static final double UNWIND_FACTOR = 0.7, WIND_FACTOR = 0.4, DEADBAND = 0.05;
    
    public static final double TOP_ROTATIONS = 80.0, SAFE_ROTATIONS = 0.0, LATCH_ROTATIONS = 120;
        
    /* current spiking check */
    public static final double SPIKE_CURRENT = 20;
    public static final double AUTOMATIC_LOWER_SPEED = 0.5;
    public static final double LOWER_TIME = 0.7;
    
    /* PID values*/
    public static final double UP_kP = 1.0, UP_kI = 0, UP_kD = 0, UP_kS = 0.00, UP_kD_TIME = 0.02;
    public static final double DOWN_kP = 0.85, DOWN_kI = 0, DOWN_kD = 0, DOWN_kD_TIME = 0.02, DOWN_kS = 0.0;
    public static final double CRUISE_VELOCITY = 100.0, ACCELERATION = 40.0;
  }
}
