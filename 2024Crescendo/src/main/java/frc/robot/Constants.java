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
  
  public static class TelescopingArm {
    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR_ID = 2;
    public static final int ENCODER_ID = 3;

    public static final double UNWIND_FACTOR = 1.0;
    public static final double WIND_FACTOR = -0.2;

    public static final double DEADBAND = 0.05;

    /* automatic climb */
    //public static final double CHAIN_HEIGHT = 1; //in meters
    public static final double MAX_WINCH_SPEED = 1; //same direction/sign as unwinding factor
    public static final double CLIMB_GEAR_RATIO = 18;
    public static final double SHAFT_DIAMETER = 0.0254; //in meters, = 1"
    public static final double TOTAL_STAGE_HEIGHT = 0.7366;// in meters, = 15+16-2 (overlap) = 29"
    public static final double MAX_WINCH_ROTATIONS = (TOTAL_STAGE_HEIGHT*CLIMB_GEAR_RATIO)/(SHAFT_DIAMETER*Math.PI); // need to check in w/ climb

    //TODO: tune values and replace
    public static final double MAX_VELOCITY = 1.00;
    public static final double MAX_ACCELERATION = 0.55; 

    public static final double MIN_OUTPUT_RANGE = -0.2;
    public static final double MAX_OUTPUT_RANGE = 0.2;

    // TODO: tune pid values
    public static final double kP = 0.85;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kD_TIME = 0.02; 
  }
}
