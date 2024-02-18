// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import java.lang.Object;
//import com.ctre.phoenix6.jni.CtreJniWrapper;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.*; 

public class KrakenShooter extends SubsystemBase {

  private TalonFX topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

  /** Creates a new KrakenShooter. */
  public KrakenShooter() {

    topLeftMotor = new TalonFX(Constants.KrakenShooter.TOP_LEFT_MOTOR_ID);
    topRightMotor = new TalonFX(Constants.KrakenShooter.TOP_RIGHT_MOTOR_ID);
    bottomLeftMotor = new TalonFX(Constants.KrakenShooter.BOTTOM_LEFT_MOTOR_ID);
    bottomRightMotor = new TalonFX(Constants.KrakenShooter.BOTTOM_RIGHT_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    configMotors(topLeftMotor, config);
    configMotors(topRightMotor, config);
    configMotors(bottomLeftMotor, config);
    configMotors(bottomRightMotor, config);
  }

  private void configMotors(TalonFX motor, TalonFXConfiguration config){
    config.Slot0.kP = Constants.KrakenShooter.kP;
    config.Slot0.kI = Constants.KrakenShooter.kI;
    config.Slot0.kD = Constants.KrakenShooter.kD;

    motor.setNeutralMode(NeutralModeValue.Coast);
    motor.getConfigurator().apply(config);
  }


  public void runKrakenShooter(double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    topLeftMotor.setControl(new VelocityVoltage(-topVelocity, -topAcceleration, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
    bottomLeftMotor.setControl(new VelocityVoltage(-bottomVelocity, -bottomAcceleration, false, 0.0, 0, false, false, false));
    bottomRightMotor.setControl(new VelocityVoltage(bottomVelocity, bottomAcceleration, false, 0.0, 0, false, false, false));
    //Kraken RPM 6000
  }

  public void stopMotors () {
    topLeftMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomLeftMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomRightMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
