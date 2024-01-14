// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private CANSparkMax leftMotor, rightMotor;
  private double factor;
  
  /**
   * Constructor for Climb class
   * <p> Initializing and configuring motors for telescoping rods
   * <p> Initializing moveWinch factor for changing speed of rods
   */
  public Climb() {
    leftMotor = new CANSparkMax(Constants.Climb.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.Climb.RIGHT_MOTOR_ID, MotorType.kBrushless);

    //TODO: add configuration?
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    factor = 0.0;
  }

  /** 
   * Sets speed of leftMotor and rightMotor to speeds of inputs multiplied by factor
   * @param leftSpeed - left motor speed
   * @param rightSpeed - right motor speed
   */
  public void moveWinch(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed){
    leftMotor.set(leftSpeed.getAsDouble() * factor);
    rightMotor.set(rightSpeed.getAsDouble() * factor);
  }

  /** 
   * Change factor for winch speed
   * @param newFactor - new value to mutiply to speed for different points in match
   */
  public void changeFactor(double newFactor){
    factor = newFactor;
  }

  /**
   * Displays current winch factor and boolean showing if climb is winding on Shuffleboard
   * @param tab - ShuffleboardTab to add values to 
   */
  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Current Winch Factor", () -> factor);
    tab.addBoolean("Is Winch Winding?", () -> (factor == Constants.Climb.WIND_FACTOR));
  }
  
  @Override
  public void periodic() {}
}