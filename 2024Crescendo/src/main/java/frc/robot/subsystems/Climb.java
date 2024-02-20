// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private TalonFX motor;
  private TalonFXConfiguration config;
  private MotionMagicVoltage request;
  private boolean manualMode = false, isRight;
  
  /**
   * Creates new Climb
   * <p> Initializing and configuring motor for Motion Magic
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public Climb(int motorID) {
    motor = new TalonFX(motorID);
    config = new TalonFXConfiguration();
    request = new MotionMagicVoltage(0, false, 0.0, 0, false, false, false);
    
    isRight = motorID == Constants.Climb.RIGHT_MOTOR_ID;
    if(isRight) motor.setInverted(true);

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
    motor.setPosition(Constants.Climb.TOP_ROTATIONS);
  }

  /**
   * Moves robot to new height using Motion Magic
   * @param pos - desired position to go to in rotations
   * @param holdingRobot - whether holding robot weight, used to configure PID accordingly
   */
  public void moveTo(double pos, boolean holdingRobot) {
    if(!manualMode){
      if(holdingRobot) motor.setControl(request.withPosition(pos).withSlot(1));
      else motor.setControl(request.withPosition(pos).withSlot(0));
    }
  }
  
  /**
   * If in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * <p> If winding, encoder position must be above "safe" position to run. Otherwise, motor stops
   * <p> If right climb, flip sign of speed
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if(manualMode){
      if(speed.getAsDouble() < 0) motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
      else motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
    }
  }
  
  /**
   * Lower motor at set speed if not in manual mode
   * @param speed - speed of motor
   */
  public void lowerAt(double speed){
    if(!manualMode) motor.set(speed * Constants.Climb.WIND_FACTOR);
    else stop();
  }
  
  /**
   * Changes manualMode field value to opposite of current value
   * <p> If switching from automatic to manual mode, stops motor
   * @return updated status of manualMode
   */
  public boolean switchMode() {
    if(!manualMode) motor.stopMotor();
    manualMode = !manualMode;
    return manualMode;
  }

  /** Stops climb motor */
  public void stop() {
    motor.stopMotor();
  }

  /** @return motor position as double */
  public double getMotorPosition(){
    return motor.getRotorPosition().getValueAsDouble();
  }
  
  /**
   * Displays boolean for mode status and values of relative and absolute encoders on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    if(isRight) tab.addDouble("Right Relative Encoder", () -> motor.getPosition().getValueAsDouble());
    else tab.addDouble("Left Relative Encoder", () -> motor.getPosition().getValueAsDouble());
    
    if(isRight) tab.addBoolean("Manual Mode?", () -> manualMode);
  }
  
  @Override
  public void periodic() {}
}