// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private SparkMax leftMotor, rightMotor;
  private DigitalInput sensor;

  /** Create new Feeder */
  public Feeder() {
    leftMotor = new SparkMax (Constants.Feeder.LEFT_ID, MotorType.kBrushless); 
    rightMotor = new SparkMax(Constants.Feeder.RIGHT_ID, MotorType.kBrushless);
    sensor = new DigitalInput(Constants.Feeder.BEAM_ID);

    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(false);

    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  
  /**
   * Runs feeder motors
   * @param leftSpeed
   * @param rightSpeed
   */
  public void feedFeeder(double speed){
    leftMotor.set(speed);
    rightMotor.set(speed);
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
    tab.addBoolean("Beam Broken?", ()->isBroken());
  }

  @Override
  public void periodic() {}
}