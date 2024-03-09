// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
  private TalonFX topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    topLeftMotor = new TalonFX(Constants.Shooter.TOP_LEFT_MOTOR_ID);
    topRightMotor = new TalonFX(Constants.Shooter.TOP_RIGHT_MOTOR_ID);
    bottomLeftMotor = new TalonFX(Constants.Shooter.BOTTOM_LEFT_MOTOR_ID);
    bottomRightMotor = new TalonFX(Constants.Shooter.BOTTOM_RIGHT_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    configMotor(topLeftMotor, config);
    configMotor(topRightMotor, config);
    configMotor(bottomLeftMotor, config);
    configMotor(bottomRightMotor, config);
  }

  /**
   * Configure motor, add PID
   * @param motor  - motor to configure
   * @param config - configuration to use
   */
  private void configMotor(TalonFX motor, TalonFXConfiguration config) {
    config.Slot0.kP = Constants.Shooter.kP;
    config.Slot0.kI = Constants.Shooter.kI;
    config.Slot0.kD = Constants.Shooter.kD;
    config.Slot0.kV = Constants.Shooter.kV;
    config.Slot0.kA = Constants.Shooter.kA;
    config.Slot0.kI = Constants.Shooter.kI;

    /*config.MotionMagic.MotionMagicCruiseVelocity = 3; //rps (4)
    config.MotionMagic.MotionMagicAcceleration = 10; //rps/s
    config.MotionMagic.MotionMagicJerk = 0;*/

    motor.setNeutralMode(NeutralModeValue.Coast);
    motor.getConfigurator().apply(config);
    
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
  }

  /**
   * Run all shooter motors at inputted velocities/accelerations
   * @param topVelocity
   * @param topAcceleration
   * @param bottomVelocity
   * @param bottomAcceleration
   */
  public void runNoPID(double topSpeed, double bottomSpeed) {
    topLeftMotor.set(-topSpeed);
    topRightMotor.set(topSpeed);
    bottomLeftMotor.set(-bottomSpeed);
    bottomRightMotor.set(bottomSpeed);
  }

  public void runShooter(double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    topLeftMotor.setControl(new VelocityVoltage(-topVelocity, -topAcceleration, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
    bottomLeftMotor
        .setControl(new VelocityVoltage(-bottomVelocity, -bottomAcceleration, false, 0.0, 0, false, false, false));
    bottomRightMotor
        .setControl(new VelocityVoltage(bottomVelocity, bottomAcceleration, false, 0.0, 0, false, false, false));
  }

  /*public void runTopPID(double topVelocity, double topAcceleration, double bottomSpeed) {
    topLeftMotor.setControl(new VelocityVoltage(-topVelocity, -topAcceleration, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
    bottomLeftMotor.set(-bottomSpeed);
    bottomRightMotor.set(bottomSpeed);
  }*/

  /**
   * Stops all shooter motors
   */
  public void stopMotors() {
    /*topLeftMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomLeftMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomRightMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));*/

    topLeftMotor.stopMotor();
    topRightMotor.stopMotor();
    bottomLeftMotor.stopMotor();
    bottomRightMotor.stopMotor();
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Average Velocity", ()->getAverageVelocity());
    tab.addDouble("Top Left", ()->getMotorVelocity(topLeftMotor));
    tab.addDouble("Top Right", ()->getMotorVelocity(topRightMotor));
    tab.addDouble("Bottom Left", ()->getMotorVelocity(bottomLeftMotor));
    tab.addDouble("Bottom Right", ()->getMotorVelocity(bottomRightMotor));
  }

  public double getAverageVelocity(){
    double averageVelocity = (Math.abs(topLeftMotor.getVelocity().getValueAsDouble())+ Math.abs(topRightMotor.getVelocity().getValueAsDouble()) + Math.abs(bottomLeftMotor.getVelocity().getValueAsDouble()) + Math.abs(bottomRightMotor.getVelocity().getValueAsDouble()))/4.0;
    return averageVelocity; 
  }

  public double getMotorVelocity(TalonFX motor){
    return Math.abs(motor.getVelocity().getValueAsDouble()); 
  }

  @Override
  public void periodic() {
  }
}