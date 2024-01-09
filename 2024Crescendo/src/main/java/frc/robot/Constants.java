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
    public static final int CANANDCODER_ID = 0;
    
    public static final double POWER_kP = 0.0001;
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kP = 0.03;
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;

    public static final double SENSOR_ROTATION_TO_MOTOR_RATIO = 0.5;
    public static final double REV_TO_METERS = 0.319024 / 7.8;
    public static final double MK4I_L1_REV_TO_METERS = 0.319024 / 8.14;
    public static final double RPM_TO_MS = MK4I_L1_REV_TO_METERS / 60;

    public static final double ANALOG_SAMPLE_DEPTH = 0;

    public static final int GYRO_PORT = 0;

    public static final int FL_POWER = 7;
    public static final int FL_STEER = 5;
    public static final int FR_POWER = 6;
    public static final int FR_STEER = 4;
    public static final int BL_POWER = 11;
    public static final int BL_STEER = 9;
    public static final int BR_POWER = 10;
    public static final int BR_STEER = 8;

    public static final double SWERVE_MAX_SPEED = 3.87096;
    public static final double SWERVE_ROTATION_MAX_SPEED = 10;
    public static final double SWERVE_DEADBAND = 0.05;

    //need to check on new chassis
    public static final double WIDTH = Units.inchesToMeters(24);
    public static final double LENGTH = Units.inchesToMeters(24);

    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3.94) * Math.PI;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(-LENGTH / 2, WIDTH / 2),
        new Translation2d(LENGTH / 2, WIDTH / 2),
        new Translation2d(-LENGTH / 2, -WIDTH / 2),
        new Translation2d(LENGTH / 2, -WIDTH / 2)
    );
  }

  public static class Auto {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedDegreesPerSecond = 180;
    public static final double kMaxAngularSpeedDegreesPerSecondSquared = 180;

    public static final double kPXController =                       25;
    public static final double kPYController = 30;
    public static final double kPThetaController = 31.65;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedDegreesPerSecond, kMaxAngularSpeedDegreesPerSecondSquared);
  }
}
