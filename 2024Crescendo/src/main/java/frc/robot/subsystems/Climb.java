// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private CANSparkMax leftMotor, rightMotor;
  private double factor;
  private RelativeEncoder leftEncoder, rightEncoder;
  
  /**
   * Constructor for Climb class
   * <p> Initializing and configuring motors for telescoping rods
   * <p> Initializing moveWinch factor for changing speed of rods
   */
  public Climb() {
    leftMotor = new CANSparkMax(Constants.Climb.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.Climb.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    factor = 0.0;

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
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
   * Set either motors to MAX_WINCH_SPEED if its encoder position is below the maximum to reach the chain's height
   * <p> Otherwise, set it to 0
   * <p> Each motor functions independently
   */
  // public void winchToChainHeight(){
  //   if (leftEncoder.getPosition() < Constants.Climb.MAX_WINCH_ROTATIONS) leftMotor.set(Constants.Climb.MAX_WINCH_SPEED);
  //   else leftMotor.set(0);
    
  //   if (rightEncoder.getPosition() < Constants.Climb.MAX_WINCH_ROTATIONS) rightMotor.set(Constants.Climb.MAX_WINCH_SPEED);
  //   else rightMotor.set(0);
  // }

  /**
   * Displays current winch factor and boolean showing if climb is winding on Shuffleboard
   * @param tab - ShuffleboardTab to add values to 
   */
  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Current Winch Factor", () -> factor);
    tab.addBoolean("Is Winch Winding?", () -> (factor == Constants.Climb.WIND_FACTOR));
    
    //untested: update values from Shuffleboard
    GenericEntry factorEntry = tab.add("Unwind Factor", Constants.Climb.UNWIND_FACTOR).getEntry();
    factorEntry.setDouble(factor);
  }
  
  @Override
  public void periodic() {}
}