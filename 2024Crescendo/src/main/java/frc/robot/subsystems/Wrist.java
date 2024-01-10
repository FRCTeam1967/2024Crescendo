// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private CANSparkMax wristMotor;
  private WPI_CANCoder absEncoder;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(1.75,0.75);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private SparkPIDController pidController;
  private double desiredAngle;

  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);

  public Wrist() {
    wristMotor = new CANSparkMax(Constants.Wrist.WRIST_MOTOR_IDX, MotorType.kBrushless);
    absEncoder = new WPI_CANCoder(0);  

    pidController = wristMotor.getPIDController();
    pidController.setP(Constants.Wrist.kP);
    pidController.setI(Constants.Wrist.kI);
    pidController.setD(Constants.Wrist.kD);
    pidController.setOutputRange(-0.2, 0.2);
    wristMotor.setSmartCurrentLimit(40);

    initEncoder();
  }

  public void initEncoder(){
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorDirection = true;
    config.unitString = "deg";
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    absEncoder.configAllSettings(config);
    absEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
  }

  public void stop() {
    wristMotor.stopMotor();
  }

  public void moveTo(double angle) {
    desiredAngle = angle;
    double currentAbsAngle = absEncoder.getAbsolutePosition();
    double degreesToMove = angle - currentAbsAngle;
    goal = new TrapezoidProfile.State(degreesToMove, 0);
  }

  public boolean isReached(){
    //return(profile.isFinished(profile.timeLeftUntil(goal.position)));
    if((Math.abs(desiredAngle - absEncoder.getAbsolutePosition())) <= 2){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Wrist.kDt, setpoint, goal);
    double degreesToRev = (setpoint.position/360) * Constants.Wrist.GEAR_RATIO;
    pidController.setReference(degreesToRev, CANSparkBase.ControlType.kPosition);
  }
}
