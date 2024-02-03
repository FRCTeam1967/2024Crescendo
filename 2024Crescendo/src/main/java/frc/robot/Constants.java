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
    public static final int LEFT_MOTOR_PDH_PORT = 7, RIGHT_MOTOR_PDH_PORT = 8;
    
    public static final double UNWIND_FACTOR = -1.0, WIND_FACTOR = 0.5;

    public static final double DEADBAND = 0.05;

    /* calculate rotations for each height for automatic climb */
    public static final double GEAR_RATIO = 18;
    public static final double SHAFT_DIAMETER = 0.0254; //in meters, = 1"
    public static final double TOP_HEIGHT = 0.6858; //total height of climb in meters, = 15+16-2 (overlap) = 29", -2" to get to our 0 position
    public static final double LATCH_HEIGHT = -0.0508; //-2"
    
    //calculate this or test & get value from shuffleboard
    public static final double TOP_ROTATIONS = (TOP_HEIGHT*Constants.Climb.GEAR_RATIO)/(Constants.Climb.SHAFT_DIAMETER*Math.PI);
    public static final double SAFE_ROTATIONS = 0.0;
    public static final double LATCH_ROTATIONS = (LATCH_HEIGHT*Constants.Climb.GEAR_RATIO)/(Constants.Climb.SHAFT_DIAMETER*Math.PI);

    /* current spiking check */
    public static final double SPIKE_CURRENT = 20;
    public static final double AUTOMATIC_LOWER_SPEED = 0.5;
    public static final double LOWER_TIME = 0.7;

    /* motion profiling */
    public static final double MAX_VELOCITY = 1.00, MAX_ACCELERATION = 0.55; 
    public static final double MIN_OUTPUT_RANGE = -0.2, MAX_OUTPUT_RANGE = 0.2;

    /* motion magic */
    public static final double CRUISE_VELOCITY = 0.4, ACCELERATION = 0.5;

    /* PID values*/
    public static final double UP_kP = 0.85, UP_kI = 0, UP_kD = 0, UP_kD_TIME = 0.02;
    public static final double DOWN_kP = 0.85, DOWN_kI = 0, DOWN_kD = 0, DOWN_kD_TIME = 0.02, DOWN_kF = 0;
  }
}
