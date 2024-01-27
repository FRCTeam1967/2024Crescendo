// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax leftMotor; 
  //rightMotor;
  //private SparkPIDController pidController;

  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor = new CANSparkMax(Constants.Shooter.SHOOTER_LEFT_MOTOR_IDX,MotorType.kBrushless);
    //rightMotor = new CANSparkMax (Constants.Shooter.SHOOTER_RIGHT_MOTOR_IDX, MotorType.kBrushless);
    /*pidController = leftMotor.getPIDController();
    pidController.setP (Constants.Shooter.kP); //make constants later
    pidController.setI (Constants.Shooter.kI);
    pidController.setD (Constants.Shooter.kD);
    pidController.setOutputRange(-0.2, 0.2);*/
    leftMotor.setSmartCurrentLimit(40);

    //rightMotor.follow(leftMotor);

  }

  public void runShooter (double speed){
    leftMotor.set(speed);
  }

  public void stopMotors () {
    leftMotor.stopMotor();
    //rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
