// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(1.00,0.55);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private CANcoder absEncoder;

  private SparkPIDController pidController;

  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new CANSparkMax(Constants.Pivot.PIVOT_ID, MotorType.kBrushless);

    pidController = pivotMotor.getPIDController();
    pidController.setP(Constants.Pivot.kP);
    pidController.setI(Constants.Pivot.kI);
    pidController.setD(Constants.Pivot.kD);
    pidController.setOutputRange(-0.2, 0.2);

    absEncoder = new CANcoder(Constants.Pivot.ENCODER_ID);

    initEncoder();
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  public void initEncoder(){
    CANcoderConfiguration config = new CANcoderConfiguration();
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    config.withMagnetSensor(magnetSensorConfigs);
    magnetSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    magnetSensorConfigs.MagnetOffset = Constants.Pivot.OFFSET;
  }

  public void moveTo(double revolutions) {
    double currentAbsRevPos = absEncoder.getAbsolutePosition().getValueAsDouble();
    double revsToMove = revolutions - currentAbsRevPos;
    goal = new TrapezoidProfile.State(revsToMove, 0);
  }

  public boolean isReached(){
    return(profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Pivot.kDt, setpoint, goal);
    double revs = (setpoint.position) * Constants.Pivot.GEAR_RATIO;
    pidController.setReference(revs, CANSparkBase.ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
