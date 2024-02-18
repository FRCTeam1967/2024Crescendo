// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;

  /**
   * Constructor for Intake class
   * <p> Initializing and configuring motors
   */
  public Intake(int motorID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
  }

  /**
   * Sets speed of motor to speed of input
   * @param speed - motor speed
   */
  public void runMotors(double speed){
    motor.set(speed);
  }

  public void stopMotors(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {}
}