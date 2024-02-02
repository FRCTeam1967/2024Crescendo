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
    public static final int LEFT_MOTOR_ID = 30;
    public static final int LEFT_ENCODER_ID = 30;

    public static final int RIGHT_MOTOR_ID = 31;
    public static final int RIGHT_ENCODER_ID = 31;

    public static final int LEFT_MOTOR_PDH_PORT = 7;
    public static final int RIGHT_MOTOR_PDH_PORT = 8;
    
    public static final double UNWIND_FACTOR = -1.0;
    public static final double WIND_FACTOR = 0.5;

    public static final double DEADBAND = 0.05;

    /* automatic climb */
    public static final double CLIMB_GEAR_RATIO = 20.25/1;
    public static final double SHAFT_DIAMETER = 0.0254; //in meters, = 1"
    public static final double TOTAL_STAGE_HEIGHT = 0.7366;// in meters, = 15+16-2 (overlap) = 29"
    public static final double MAX_WINCH_ROTATIONS = (TOTAL_STAGE_HEIGHT*CLIMB_GEAR_RATIO)/(SHAFT_DIAMETER*Math.PI);
    public static final double LOW_WINCH_ROTATIONS = ((TOTAL_STAGE_HEIGHT/4)*CLIMB_GEAR_RATIO)/(SHAFT_DIAMETER*Math.PI); //quarter of the way up
    public static final double LATCH_POSITION_ROTATIONS = 0.0;
    
    public static final double SPIKE_CURRENT = 20;
    public static final double AUTOMATIC_LOWER_SPEED = 0.5;
    public static final double LOWER_TIME = 0.7;
  
    // TODO: tune pid values
    public static final double kP = 0.85;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kD_TIME = 0.02;

    public static final double CRUISE_VELOCITY = 0.4;
    public static final double ACCELERATION = 0.5;

    public static final int kPIDLoopIdx = 0;
  }
}
