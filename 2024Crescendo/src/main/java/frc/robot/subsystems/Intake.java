// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX motor;

  /**
   * Constructor for Intake class
   * <p> Initializing and configuring motors
   */
  public Intake(int motorID) {
    motor = new TalonFX(motorID);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    motor.getConfigurator().apply(config);
  }

  /**
   * Sets speed of motor to speed of input
   * @param speed - motor speed
   */
  public void runMotors(double speed){
    motor.set(speed);
  }

  @Override
  public void periodic() {}
}