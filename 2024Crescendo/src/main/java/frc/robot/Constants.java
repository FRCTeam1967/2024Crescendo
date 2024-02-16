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

  public static class Feeder{
    public static final int LEFT_ID = 31;
    public static final int RIGHT_ID = 32; //tbd
    //public static final double kD_TIME = 0.02;

    public static final double FEED_TIME = 5.0;
    public static final double FEED_SPEED = 0.3;
  }

  public static class KrakenShooter {
    public static final double MIN_OUTPUT_RANGE = 1; //will tune both of these later
    public static final double MAX_OUTPUT_RANGE = -1;

    public static final int TOP_LEFT_MOTOR_ID = 28; //tbd
    public static final int TOP_RIGHT_MOTOR_ID = 27; //tbd
    public static final int BOTTOM_LEFT_MOTOR_ID = 25; //tbd
    public static final int BOTTOM_RIGHT_MOTOR_ID = 26; //tbd


    public static final double TOP_LEFT_SPEED = 0.15;
    public static final double TOP_RIGHT_SPEED = 0.15;
    public static final double BOTTOM_LEFT_SPEED = 0.15;
    public static final double BOTTOM_RIGHT_SPEED = 0.15;

    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double TOP_VELOCITY = 100;
    public static final double TOP_ACCELERATION = 75;

    public static final double BOTTOM_VELOCITY = 100;
    public static final double BOTTOM_ACCELERATION = 75;

  }

  

  public static class Shooter {
    public static final int FRONT_LEFT_MOTOR_IDX = 1; //tbd
    public static final int FRONT_RIGHT_MOTOR_IDX = 2; //tbd

    public static final int BACK_LEFT_MOTOR_IDX = 3; //tbd
    public static final int BACK_RIGHT_MOTOR_IDX = 4; //tbd

    public static final double FRONT_SPEED = 0.3;
    public static final double BACK_SPEED = 0.3;

    public static final double SHOOT_TIME = 5.0;
  }

  public static class Vision {
    public static final double DEGREE_ERROR = 2.0;
  }

  public static final class Intake {
    public static final int MOTOR_ID = 8;
        
    public static final double INTAKE_SPEED = -0.5;
    public static final double EJECT_SPEED = -0.25;
  }

  public static class Pivot {
    public static final int PIVOT_ID = 30; 
    public static final int ENCODER_ID = 32;

    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kD_TIME = 0.02;

    public static final double GEAR_RATIO = 50/1; //Mech said 50/1

    public static final double STARTING_ANGLE = 45;

    public static final int OFFSET = 0;
    public static final int NEO_TICKS_PER_REVOLUTION = 42;
    
    public static final double ABSOLUTE_TICKS_PER_REVOLUTION = 4096;
    public static final double CONVERSION_FACTOR = 1.0/360.0;
    
    public static final double TEST_20 = 20 * Constants.Pivot.CONVERSION_FACTOR;
    public static final double TEST_70 = 70 * Constants.Pivot.CONVERSION_FACTOR;

    public static final double INTAKE_SAFE = 114 * Constants.Pivot.CONVERSION_FACTOR;
    public static final double INTAKE_DOWN = 10 * Constants.Pivot.CONVERSION_FACTOR;

    


    public static final double DEGREE_0 = 0;
  }
}
