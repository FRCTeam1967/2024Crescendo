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

  private CANSparkMax frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    frontLeftMotor = new CANSparkMax(Constants.Shooter.FRONT_LEFT_MOTOR_IDX,MotorType.kBrushless);
    frontRightMotor = new CANSparkMax (Constants.Shooter.FRONT_RIGHT_MOTOR_IDX, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax (Constants.Shooter.BACK_LEFT_MOTOR_IDX, MotorType.kBrushless);
    backRightMotor = new CANSparkMax (Constants.Shooter.BACK_RIGHT_MOTOR_IDX, MotorType.kBrushless);

    
    frontLeftMotor.setSmartCurrentLimit(40);
    frontRightMotor.setSmartCurrentLimit(40);
    backLeftMotor.setSmartCurrentLimit(40);
    backRightMotor.setSmartCurrentLimit(40);
  }

  public void runShooter (double frontSpeed, double backSpeed){
    frontLeftMotor.set(frontSpeed);
    frontRightMotor.set(frontSpeed);
    backLeftMotor.set(backSpeed);
    backRightMotor.set(backSpeed);
  }

  public void stopMotors () {
    frontLeftMotor.stopMotor();
    frontRightMotor.stopMotor();
    backLeftMotor.stopMotor();
    backRightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
