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
    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR_ID = 2;

    public static final double UNWIND_FACTOR = 1.0;
    public static final double WIND_FACTOR = -0.2;

    public static final double DEADBAND = 0.05;

    //automatically shooting rods up to chain height
    public static final double CHAIN_HEIGHT = 1; //in meters
    //public static final double SHAFT_RADIUS = 0.01; //in meters, NEO shaft radius is 0.004 but also need to account for string
    //public static final double MAX_WINCH_ROTATIONS = CHAIN_HEIGHT/(2*Math.PI*SHAFT_RADIUS);
    public static final double MAX_WINCH_SPEED = 1; //same direction/sign as unwinding factor
    public static final double CLIMB_GEAR_RATIO = 18;
    public static final double SHAFT_RADIUS = 0.0254; // spool is 1 inch in diameters
    public static final double STAGE_HEIGHT = 0.7366;// in meters, 29 inches accounts for 2in overlap
    public static final double MAX_WINCH_ROTATIONS = STAGE_HEIGHT/(SHAFT_RADIUS*Math.PI)*CLIMB_GEAR_RATIO;
  }
}
