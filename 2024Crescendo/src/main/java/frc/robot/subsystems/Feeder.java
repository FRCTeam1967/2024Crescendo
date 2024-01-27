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

  private CANSparkMax feederMotor;
  private SparkPIDController pidController;

  /** Creates a new Feeder. */
  public Feeder() {
    feederMotor = new CANSparkMax (Constants.Feeder.FEEDER_ID, MotorType.kBrushless); //make constant later
    feederMotor.setSmartCurrentLimit(40);
  }
  
  public void feedFeeder (double speed){
    feederMotor.set(speed);
  }

  public void stopFeeder () {
    feederMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
