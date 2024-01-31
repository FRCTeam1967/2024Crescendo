// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  private CANSparkMax leftMotor, rightMotor;

  /** Creates a new Feeder. */
  public Feeder() {
    leftMotor = new CANSparkMax (Constants.Feeder.LEFT_ID, MotorType.kBrushless); 
    rightMotor = new CANSparkMax(Constants.Feeder.RIGHT_ID, MotorType.kBrushless);
    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);
  }
  
  public void feedFeeder (double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void stopFeeder () {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
