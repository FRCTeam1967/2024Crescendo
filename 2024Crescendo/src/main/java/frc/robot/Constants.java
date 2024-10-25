// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Xbox { 
    public static final int DRIVER_CONTROLLER_PORT = 0, OPERATOR_CONTROLLER_PORT = 1;
    public static final double DRIVER_DEADBAND = 0.02;
  }

  public static class AmpBar { //TODO: change values!
    public static final int AMP_BAR_ID = 24;
    public static final double kP = 1, kI = 0, kD = 0, kD_TIME = 0.02; 
    public static final double GEAR_RATIO = 10/1.0;

    public static final double MAX_VELOCITY = 165;
    public static final double MAX_ACCELERATION = 180;

    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;
    public static final double AMP_SAFE = 4 * Constants.AmpBar.DEGREES_TO_REVOLUTIONS;
    public static final double AMP_UP = 98 * Constants.AmpBar.DEGREES_TO_REVOLUTIONS; // 94
  }
  
  public static class Climb {
    public static final int LEFT_MOTOR_ID = 30, RIGHT_MOTOR_ID = 31;
    public static final int LEFT_DIGITAL_INPUT_ID = 9, RIGHT_DIGITAL_INPUT_ID = 0;
    public static final double UNWIND_FACTOR = 1.0, WIND_FACTOR = 0.5, DEADBAND = 0.05;

    public static final double LOWER_SPEED = 0.65, TOP_ROTATIONS = -110.0;
    public static final double CURRENT_LIMIT = 40;
    
    public static final double kP = 1.0, kI = 0, kD = 0, kS = 0.00, kD_TIME = 0.02;
    public static final double CRUISE_VELOCITY = 100.0, ACCELERATION = 40.0;
  }
  
  public static class Feeder {
    public static final int LEFT_ID = 17, RIGHT_ID = 18;
    public static final double FEED_TIME = 5.0, FEED_SPEED = -0.8, REVERSE_SPEED = 0.2;
    public static final int BEAM_ID = 8;
  }

  public static class Swerve {
    //motor/encoder ids
    public static final int FL_POWER = 7, FL_STEER = 8, FL_ENCODER = 4;
    public static final int FR_POWER = 1, FR_STEER = 2, FR_ENCODER = 1;
    public static final int BL_POWER = 5, BL_STEER = 6, BL_ENCODER = 3;
    public static final int BR_POWER = 3, BR_STEER = 4, BR_ENCODER = 2;

    public static final double FL_OFFSET = -136.40625/360; //-131.3059375/360;
    public static final double FR_OFFSET = -0.3515625/360;//3.427734375/360;
    public static final double BL_OFFSET = 115.048828125/360; //117.59765625;
    public static final double BR_OFFSET = -17.05078125/360; //-17.138671875/360;
    public static final int PIGEON_GYRO = 9;

    // public static final double FL_OFFSET = -132.01171875/360; //-134.560546875/360;
    // public static final double FR_OFFSET = 4.39453125/360;//107.40234375/360;
    // public static final double BL_OFFSET = 117.421875/360; //116.630859375/360;
    // public static final double BR_OFFSET = -17.2265625/360; //-17.40234375/360;

    //janky offsets
    // public static final double FL_OFFSET = 171.507813/360;
    // public static final double FR_OFFSET = -59.050391/360;
    // public static final double BL_OFFSET = 119.619141/360;
    // //public static final double BR_OFFSET = (-150.820313-180)/360;
    // public static final double BR_OFFSET = -150.820313/360;

    // public static final double FL_OFFSET = -10.0195/360;
    // public static final double FR_OFFSET = 117.6855/360;
    // public static final double BL_OFFSET = -52.7343/360;
    // public static final double BR_OFFSET = 27.1582/360;


    //pid values
    public static final double POWER_kS = 0.14;//0.14; 
    public static final double POWER_kV = 12/(100/8.14); //1.25; 
    public static final double POWER_kA = 0; 
    public static final double POWER_kP = POWER_kV * 0.8; //0.25;
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kS = 0; //0.1
    public static final double STEER_kV = 0; //30
    public static final double STEER_kA = 0; // 15
    public static final double STEER_kP = 12; //100
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0.5;

    //gear ratios + meter conversions
    public static final double STEER_GEAR_RATIO = 150.0/7.0;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final double MK4I_L1_REV_TO_METERS = WHEEL_CIRCUMFERENCE;
    public static final double RPM_TO_MPS = MK4I_L1_REV_TO_METERS / 60.0;
    public static final double SENSOR_ROTATION_TO_MOTOR_RATIO = STEER_GEAR_RATIO;
    public static final double REVERSE_OFFSET = Units.inchesToMeters(2.0) * Math.PI;
    public static final double METERS_TO_ENC_COUNT = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    

    //distances/measurements
    public static final double SWERVE_MAX_SPEED = 4.1695; //m/s
    public static final double WIDTH = Units.inchesToMeters(23), LENGTH = Units.inchesToMeters(23);
    public static final double SWERVE_AMP_OFFSET = 0.3083496; //rotations of encoder
    public static final double AMP_REVERSE_JS_INPUT = 0.4; //joystick input

    //max speeds
    public static final double ROTATION_CIRCLE_CIRCUMFERENCE = (WIDTH / Math.sqrt(2.0)) * 2.0 * Math.PI;
    public static final double SWERVE_ROTATION_MAX_SPEED_IN_RAD = (SWERVE_MAX_SPEED / ROTATION_CIRCLE_CIRCUMFERENCE) * 2.0 * Math.PI; 

    //kinematics
    public static Translation2d m_frontLeftLocation = new Translation2d(LENGTH / 2, WIDTH / 2);
    public static Translation2d m_frontRightLocation = new Translation2d(LENGTH / 2, -WIDTH / 2);
    public static Translation2d m_backLeftLocation = new Translation2d(-LENGTH / 2, WIDTH / 2);
    public static Translation2d m_backRightLocation = new Translation2d(-LENGTH / 2, -WIDTH / 2);

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );
    
    public static final double SWERVE_ROTATION_TOLERANCE = 5;

    public static final TrapezoidProfile.Constraints SWERVE_ROTATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(36000, 36000);
    public static final TrapezoidProfile.Constraints SWERVE_TRANSLATION_PID_CONSTRAINTS = new TrapezoidProfile.Constraints(15, 3);
  }

  public static class Vision {
    public static final double DEGREE_ERROR = 4.0;

    public static final double LIMELIGHT_ANGLE_DEGREES = 42; //need to verify
    public static final double LIMELIGHT_HEIGHT_INCHES = 25.125; //need to verify
    public static final double TARGET_HEIGHT_INCHES = 54; //need to verify
  }

  public static class Auto {
    public static final double PIVOT_INTAKE_TIMEOUT = 2.0, PIVOT_UP_TIMEOUT = 2.0, SHOOT_SPEAKER_TIMEOUT = 3.0; //pivot timeout: 2
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
    public static final double kMaxAngularSpeedDegreesPerSecond = 180;
    public static final double kMaxAngularSpeedDegreesPerSecondSquared = 180;

    public static final double APRIL_TAG_HEIGHT_INCHES = 48.125; //need to verify
    public static final double LIMELIGHT_ANGLE_DEGREES = 45.0; //need to measure
    public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 37.0; //need to measure

    public static final double kPXController = 25, kPYController = 30, kPThetaController = 6.5; //7.5

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(kMaxAngularSpeedDegreesPerSecond, kMaxAngularSpeedDegreesPerSecondSquared);
    public static final double RADIANS_TO_DEGREES = 180.0/Math.PI;
  } 

  public static final class Intake {
    public static final int MOTOR_ID = 13;
    public static final double INTAKE_ROLLER_SPEED = -0.6;
  }

  public static class Shooter {
    public static final int TOP_LEFT_MOTOR_ID = 20, TOP_RIGHT_MOTOR_ID = 21;
    public static final int BOTTOM_LEFT_MOTOR_ID = 19, BOTTOM_RIGHT_MOTOR_ID = 22;

    public static final double kP = 0.32, kI = 0, kD = 0, kV = 0.07, kA = 0.06;
    
    public static final double A_TOP_SPEED = 0.162;
    public static final double A_BOTTOM_SPEED = 0.162;
    
    public static final double AMP_TOP_VELOCITY = 10, AMP_TOP_ACCELERATION = 12;
    public static final double AMP_BOTTOM_VELOCITY = 10, AMP_BOTTOM_ACCELERATION = 12;
    
    public static final double S_TOP_SPEED = 1;
    public static final double S_BOTTOM_SPEED = 1;
    
    public static final double SPEAKER_TOP_VELOCITY = 90, SPEAKER_TOP_ACCELERATION = 80; // 80, 70
    public static final double SPEAKER_BOTTOM_VELOCITY = 60, SPEAKER_BOTTOM_ACCELERATION = 50;
    
    public static final double THRESHOLD_SPEED = 80;
  }
  
  public static class Pivot {
    public static final int PIVOT_ID = 14, ENCODER_ID = 23;
    public static final double GEAR_RATIO = 50.0/1.0;

    public static final double kP = 1, kI = 0, kD = 0, kD_TIME = 0.02;

    public static final double MAX_VELOCITY = 120;
    public static final double MAX_ACCELERATION = 150;
    
    public static final double DEGREES_TO_REVOLUTIONS = 1.0/360.0;
    public static final double INTAKE_SAFE = 110 * Constants.Pivot.DEGREES_TO_REVOLUTIONS;
    public static final double INTAKE_DOWN = 6 * Constants.Pivot.DEGREES_TO_REVOLUTIONS;
  }

  public static class ExperimentalFeatures {
    public static final boolean useCosineCompensation = true; //TODO: test on tuesday if driving is okay with this true
    public static final boolean disableRotationWhenNotMoving = false;
    public static final boolean applyDriverDeadband = false;

    // Setting this to true because that's how the code works, but there's no reason to do this AFAICT, and
    // it causes unnecessary CAN traffic. Each call to the 4 modules is BLOCKING. CTRE says:
    // "We recommend that users avoid calling this API periodically"
    // So this should really be set to FALSE.
    public static final boolean applyNeutralModeConstantlyInAuto = false;
  }
}