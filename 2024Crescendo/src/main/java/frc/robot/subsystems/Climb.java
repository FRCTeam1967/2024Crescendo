// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private TalonFX motor;
  private TalonFXConfiguration config;
  private MotionMagicVoltage request;
  private boolean isRight;
  private DigitalInput sensor;

  public boolean isZero;
  
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
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //was changed SVR practice matches, theoretically should be false
    } else {
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //was changed SVR practice matches, theoretically should be false
      sensor = new DigitalInput(Constants.Climb.LEFT_DIGITAL_INPUT_ID);
    }

    motor.setNeutralMode(NeutralModeValue.Brake);

    config.Slot0.kP = Constants.Climb.UP_kP;
    config.Slot0.kI = Constants.Climb.UP_kI;
    config.Slot0.kD = Constants.Climb.UP_kD;
    config.Slot0.kS = Constants.Climb.UP_kS;

    config.Slot1.kP = Constants.Climb.DOWN_kP;
    config.Slot1.kI = Constants.Climb.DOWN_kI;
    config.Slot1.kD = Constants.Climb.DOWN_kD;
    config.Slot1.kS = Constants.Climb.DOWN_kS;

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
   * @param holdingRobot - whether holding robot weight, used to configure PID accordingly
   */
  public void moveTo(double pos, boolean holdingRobot) {
    if(holdingRobot) motor.setControl(request.withPosition(pos).withSlot(1));
    else motor.setControl(request.withPosition(pos).withSlot(0));
  }
  
  /**
   * JUST FOR TESTING: if in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if(speed.getAsDouble() < 0) motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
    else motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
  }

  public double getPosition(){
    return motor.getPosition().getValueAsDouble();
  }
  
  /**
   * Lower motor at set speed if not in manual mode
   * @param speed - speed of motor
   */
  public void lowerAt(double speed){
    motor.set(speed * Constants.Climb.WIND_FACTOR);
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
    /*if (!getSensorValue()){
      motor.setPosition(0);
      isZero = true;
    }else{
      SmartDashboard.putString("zero position?", "no");
      isZero = false;
    }*/
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