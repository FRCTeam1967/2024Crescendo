// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Xbox {
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
  }
  
  public static class Climb {
    public static final int LEFT_MOTOR_ID = 30, RIGHT_MOTOR_ID = 31, LEFT_DIGITAL_INPUT_ID = 1;
    public static final int LEFT_MOTOR_PDH_PORT = 18, RIGHT_MOTOR_PDH_PORT = 5, RIGHT_DIGITAL_INPUT_ID = 0;
    
    public static final double UNWIND_FACTOR = 0.7, WIND_FACTOR = 0.4, DEADBAND = 0.05;
    
    public static final double TOP_ROTATIONS = -130.0, SAFE_ROTATIONS = 0.0, LATCH_ROTATIONS = 120;
    
    public static final double LOWER_SPEED = 0.8;
    public static final double CURRENT_LIMIT = 40;
        
    /* PID values*/
    public static final double UP_kP = 1.0, UP_kI = 0, UP_kD = 0, UP_kS = 0.00, UP_kD_TIME = 0.02;
    public static final double DOWN_kP = 0.85, DOWN_kI = 0, DOWN_kD = 0, DOWN_kD_TIME = 0.02, DOWN_kS = 0.0;
    public static final double CRUISE_VELOCITY = 100.0, ACCELERATION = 40.0;
  }
  
  public static class Feeder {
    public static final int LEFT_ID = 17, RIGHT_ID = 18;
    public static final double FEED_TIME = 5.0, FEED_SPEED = -0.8, REVERSE_SPEED = 0.2;
  }

  public static class Swerve {
    public static final double POWER_kS = 0.2;//0.14; //0.14 best
    public static final double POWER_kV = 8;//0.9; //1.8, 1., 0.8 //in volts
    public static final double POWER_kA = 0.1;//0.1; 
    public static final double POWER_kP = 0.01;//0.01; //0.2 //in rotations
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kS = 0.1;//0.05;//0.15;//0.13; // 0.6, 0.8, 0.7, 0.15 (still moving)
    public static final double STEER_kV = 30;//35;//2.1; // 0.5, 0, 0.1, 1
    public static final double STEER_kA = 15;//15;//0.1; //typically small (Ryan)
    public static final double STEER_kP = 12.5;//12.5;//3; // 1.2, 0.8, 0.1 (1868: 2.4)
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;

    public static final int pigeonID = 1;

    public static final int climbLeftEncoderIdx = 30;
    public static final int climbRightEncoderIdx = 31;

    //0.319024 = circumference in meters
    //12.8:1 = rotor to sensor ratio
    public static final double STEER_GEAR_RATIO = 150/7;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
    public static final double MK4I_L1_REV_TO_METERS = WHEEL_CIRCUMFERENCE;
    public static final double RPM_TO_MPS = MK4I_L1_REV_TO_METERS / 60;
    public static final double SENSOR_ROTATION_TO_MOTOR_RATIO = STEER_GEAR_RATIO;

    public static final double ANALOG_SAMPLE_DEPTH = 0;

    public static final double REVERSE_OFFSET_REVS = Units.inchesToMeters(0.15);

    public static final int FL_POWER = 7;
    public static final int FL_STEER = 8;
    public static final int FL_ENCODER = 4;

    public static final int FR_POWER = 1;
    public static final int FR_STEER = 2;
    public static final int FR_ENCODER = 1;

    public static final int BL_POWER = 5;
    public static final int BL_STEER = 6;
    public static final int BL_ENCODER = 3;

    public static final int BR_POWER = 3;
    public static final int BR_STEER = 4;
    public static final int BR_ENCODER = 2;

    public static final double SWERVE_MAX_SPEED = 4.1695;//0.3; //4.1695 mps //3 for driver
    public static final double WIDTH = Units.inchesToMeters(23);
    public static final double LENGTH = Units.inchesToMeters(23);

    public static final double SWERVE_AMP_OFFSET = 0.3083496;

    public static final double ROTATION_CIRCLE_CIRCUMFERENCE = (WIDTH / Math.sqrt(2)) * 2 * Math.PI;

    public static final double SWERVE_ROTATION_MAX_SPEED_IN_RAD = (2 / WHEEL_CIRCUMFERENCE) * 2 * Math.PI; 
    public static final double SWERVE_DEADBAND = 0.05;

    public static Translation2d m_frontLeftLocation = new Translation2d(LENGTH / 2, WIDTH / 2);
    public static Translation2d m_frontRightLocation = new Translation2d(LENGTH / 2, -WIDTH / 2);
    public static Translation2d m_backLeftLocation = new Translation2d(-LENGTH / 2, WIDTH / 2);
    public static Translation2d m_backRightLocation = new Translation2d(-LENGTH / 2, -WIDTH / 2);


    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
      //fl
        m_frontLeftLocation,
      //fr
        m_frontRightLocation,
      //bl
        m_backLeftLocation,
      //br
        m_backRightLocation
    );

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );


    public static final double SWERVE_ROTATION_TOLERANCE = 5;

    public static final TrapezoidProfile.Constraints SWERVE_ROTATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(36000, 36000);
    public static final TrapezoidProfile.Constraints SWERVE_TRANSLATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(15, 3);
  }

  public static class Vision {
    public static final double DEGREE_ERROR = 5.0;
  }

  public static class Auto {
    public static final double PIVOT_INTAKE_TIMEOUT = 2.0, PIVOT_UP_TIMEOUT = 2.0, SHOOT_SPEAKER_TIMEOUT = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
    public static final double kMaxAngularSpeedDegreesPerSecond = 180;
    public static final double kMaxAngularSpeedDegreesPerSecondSquared = 180;

    public static final double kPXController = 25;
    public static final double kPYController = 30;
    public static final double kPThetaController = 7.5;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(kMaxAngularSpeedDegreesPerSecond, kMaxAngularSpeedDegreesPerSecondSquared);
    public static final double RADIANS_TO_DEGREES = 57.29578;
  } 

  public static final class Intake {
    public static final int MOTOR_ID = 13;
    public static final double INTAKE_ROLLER_SPEED = -0.6, EJECT_ROLLER_SPEED = -0.25;
  }

  public static class Shooter {
    public static final int TOP_LEFT_MOTOR_ID = 20, TOP_RIGHT_MOTOR_ID = 21;
    public static final int BOTTOM_LEFT_MOTOR_ID = 19, BOTTOM_RIGHT_MOTOR_ID = 22;

    public static final double kP = 0.32, kI = 0, kD = 0;

    public static final double SPEAKER_TOP_VELOCITY = 1, SPEAKER_TOP_ACCELERATION = 1;
    public static final double SPEAKER_BOTTOM_VELOCITY = 1, SPEAKER_BOTTOM_ACCELERATION = 1;

    public static final double AMP_TOP_VELOCITY = 0.08, AMP_TOP_ACCELERATION = 0.08;
    public static final double AMP_BOTTOM_VELOCITY = 0.08, AMP_BOTTOM_ACCELERATION = 0.08;
  }
  
  public static class Pivot {
    public static final int PIVOT_ID = 14, ENCODER_ID = 23;
    public static final double GEAR_RATIO = 50/1;

    public static final double kP = 3, kI = 0, kD = 0, kD_TIME = 0.02;
    
    public static final double CONVERSION_FACTOR = 1.0/360.0;
    public static final double INTAKE_SAFE = 110 * Constants.Pivot.CONVERSION_FACTOR;
    public static final double INTAKE_DOWN = 6 * Constants.Pivot.CONVERSION_FACTOR;
  }
}