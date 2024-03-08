// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climb extends SubsystemBase {
  private TalonFX motor;
  private TalonFXConfiguration config;
  private MotionMagicVoltage request;
  private boolean isRight;
  private DigitalInput sensor;
  
  /**
   * Creates new Climb
   * <p> Initializing and configuring motor for Motion Magic, initializing sensor
   * @param motorID - port for motor
   */
  public Climb(int motorID) {
    motor = new TalonFX(motorID);
    config = new TalonFXConfiguration();
    request = new MotionMagicVoltage(0, false, 0.0, 0, false, false, false);
    
    isRight = (motorID == Constants.Climb.RIGHT_MOTOR_ID);
    if(isRight) {
      motor.setInverted(true);
      sensor = new DigitalInput(Constants.Climb.RIGHT_DIGITAL_INPUT_ID);
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      sensor = new DigitalInput(Constants.Climb.LEFT_DIGITAL_INPUT_ID);
    }
    
    motor.setNeutralMode(NeutralModeValue.Brake);

    config.Slot0.kP = Constants.Climb.kP;
    config.Slot0.kI = Constants.Climb.kI;
    config.Slot0.kD = Constants.Climb.kD;
    config.Slot0.kS = Constants.Climb.kS;
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Climb.CURRENT_LIMIT));
    
    motor.getConfigurator().apply(config);
  }
  
  /** Sets zero position of encoder to just above the latch position of the telescoping arm */
  public void setEncoderOffset() {
    motor.setPosition(0);
  }

  /**
   * Moves robot to new height using Motion Magic
   * @param pos - desired position to go to in rotations
   */
  public void moveTo(double pos) {
    motor.setControl(request.withPosition(pos).withSlot(0));
  }

  /** @return position of motor */
  public double getPosition(){
    return motor.getPosition().getValueAsDouble();
  }
  
  /**
   * Lower motor at set speed if not in manual mode
   * @param lower - true if lowering climb
   */
  public void runMotors(boolean lower){
    if(lower) motor.set(Constants.Climb.LOWER_SPEED);
    else motor.set(Constants.Climb.RAISE_SPEED);
  }

  /**
   * @param speed - speed of motor
   */
  public void runManual(DoubleSupplier speed) {
    double deadbandedSpeed = MathUtil.applyDeadband(speed.getAsDouble(), Constants.Climb.DEADBAND); //TODO: test if values update properly

    if(deadbandedSpeed < 0) motor.set(deadbandedSpeed * Constants.Climb.WIND_FACTOR);
    else motor.set(deadbandedSpeed * Constants.Climb.UNWIND_FACTOR);
  }

  /** Stops climb motor */
  public void stop() {
    motor.stopMotor();
  }

  /** @return sensor value (false when triggered) */
  public boolean getSensorValue() {
    return sensor.get();
  }

  public void setToZero(){
    motor.setPosition(0);
  }

  /**
   * Displays value of relative encoder on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    if(isRight){
      tab.addDouble("Right Climb Rel Encoder", () -> motor.getRotorPosition().getValueAsDouble());
      tab.addBoolean("Right Climb Sensor", () -> getSensorValue());
      tab.addDouble("Right Climb Pos", ()-> motor.getPosition().getValueAsDouble());
    } else {
      tab.addDouble("Left Climb Rel Encoder", () -> motor.getRotorPosition().getValueAsDouble());
      tab.addBoolean("Left Climb Sensor", () -> getSensorValue());
      tab.addDouble("Left Climb Pos", ()-> motor.getPosition().getValueAsDouble());
    }      
  }
  
  @Override
  public void periodic() {}
}