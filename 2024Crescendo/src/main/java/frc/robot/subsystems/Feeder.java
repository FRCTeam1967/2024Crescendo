// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private CANSparkMax leftMotor, rightMotor;
  private DigitalInput sensor;

  /** Creates new Feeder */
  public Feeder() {
    leftMotor = new CANSparkMax (Constants.Feeder.LEFT_ID, MotorType.kBrushless); 
    rightMotor = new CANSparkMax(Constants.Feeder.RIGHT_ID, MotorType.kBrushless);

    sensor = new DigitalInput(8);
  }
  
  /**
   * Runs feeder motors
   * @param leftSpeed
   * @param rightSpeed
   */
  public void feedFeeder(double leftSpeed, double rightSpeed){
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  /** Stops feeder motors */
  public void stopFeeder() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean isBroken(){
    return !(sensor.get());
  }

  public void configDashboard(ShuffleboardTab tab){
    tab.addBoolean("Beam Broken?", ()-> isBroken());
  }

  @Override
  public void periodic() {}
}