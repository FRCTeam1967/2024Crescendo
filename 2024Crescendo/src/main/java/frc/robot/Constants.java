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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {
    public static final double POWER_kS = 0.14; //0.14 best
    public static final double POWER_kV = 0.9; //1.8, 1., 0.8 //in volts
    public static final double POWER_kA = 0.1; 
    public static final double POWER_kP = 0.01; //0.2 //in rotations
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kS = 0.13; // 0.6, 0.8, 0.7, 0.15 (still moving)
    public static final double STEER_kV = 2.1; // 0.5, 0, 0.1, 1
    public static final double STEER_kA = 0.1; //typically small (Ryan)
    public static final double STEER_kP = 3; // 1.2, 0.8, 0.1 (1868: 2.4)
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;

    public static final int pigeonID = 1;

    //0.319024 = circumference in meters
    //12.8:1 = rotor to sensor ratio
    public static final double STEER_GEAR_RATIO = 150/7;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
    public static final double MK4I_L1_REV_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double RPM_TO_MPS = MK4I_L1_REV_TO_METERS / 60;
    public static final double SENSOR_ROTATION_TO_MOTOR_RATIO = STEER_GEAR_RATIO;

    public static final double ANALOG_SAMPLE_DEPTH = 0;

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

    public static final double SWERVE_MAX_SPEED = 2.085; //4.1695 mps
    public static final double WIDTH = Units.inchesToMeters(23);
    public static final double LENGTH = Units.inchesToMeters(23);

    public static final double ROTATION_CIRCLE_CIRCUMFERENCE = (WIDTH / Math.sqrt(2)) * 2 * Math.PI;

    public static final double SWERVE_ROTATION_MAX_SPEED_IN_RAD = SWERVE_MAX_SPEED/ROTATION_CIRCLE_CIRCUMFERENCE; 
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




  }

  public static class Auto {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedDegreesPerSecond = 180;
    public static final double kMaxAngularSpeedDegreesPerSecondSquared = 180;

    public static final double kPXController = 25;
    public static final double kPYController = 30;
    public static final double kPThetaController = 31.65;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedDegreesPerSecond, kMaxAngularSpeedDegreesPerSecondSquared);
    public static final double RADIANS_TO_DEGREES = 57.29578;
  }
}
