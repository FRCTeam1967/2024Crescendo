// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax topRollerMotor, bottomRollerMotor;
  private DigitalInput limitSwitch;

  /**
   * Constructor for Intake class
   * <p> Initializing and configuring motors
   */
  public Intake() {
    topRollerMotor = new CANSparkMax(Constants.Intake.TOP_ROLLER_MOTOR_ID, MotorType.kBrushless);
    bottomRollerMotor = new CANSparkMax(Constants.Intake.BOTTOM_ROLLER_MOTOR_ID, MotorType.kBrushless);
    
    limitSwitch = new DigitalInput(Constants.Intake.LIMIT_SWITCH_ID);
    
    //configuration
    topRollerMotor.restoreFactoryDefaults();
    bottomRollerMotor.restoreFactoryDefaults();
  }

  /**
   * Sets speed of topRollerMotor and bottomRollerMotor to speed of inputs
   * @param topSpeed - top roller motor speed
   * @param bottomSpeed - bottom roller motor speed
   */
  public void runMotors(double topSpeed, double bottomSpeed){
    topRollerMotor.set(topSpeed);
    bottomRollerMotor.set(bottomSpeed);
  }

  /**
   * @return value of limit switch (true if triggered)
   */
  public boolean limitSwitchTriggered(){
    return limitSwitch.get();
  }
  
  /**
   * Run intake rollers outward at slow speed for a few seconds to eject into shooter
   * <p> Timer functionality in {@link frc.robot.RobotContainer}
   */
  public void ejectIntoShooter(){
    topRollerMotor.set(Constants.Intake.EJECT_TOP_ROLLER_SPEED);
    bottomRollerMotor.set(Constants.Intake.EJECT_BOTTOM_ROLLER_SPEED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
