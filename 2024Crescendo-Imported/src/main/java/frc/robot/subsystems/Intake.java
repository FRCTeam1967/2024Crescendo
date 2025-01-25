// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  private SparkMax motor;

  /** Creates new Intake */
  public Intake() {
    motor = new SparkMax(Constants.Intake.MOTOR_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets speed of motor to speed of input
   * @param speed - motor speed
   */
  public void runMotors(double speed){
    motor.set(speed);
  }
  
  /** Stops intake motor */
  public void stopMotors(){
    motor.stopMotor();
  }
  
  @Override
  public void periodic() {}
}