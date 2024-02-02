// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private TalonFX motor;
  private Canandcoder absEncoder;
  private boolean safeToClimb = false;

  /**
   * Constructor for Climb class
   * <p> Initializing and configuring motor for Motion Magic
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public Climb(int motorID, int encoderID) {
    motor = new TalonFX(motorID);
    absEncoder = new Canandcoder(encoderID);
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Climb.kP;
    config.Slot0.kI = Constants.Climb.kI;
    config.Slot0.kD = Constants.Climb.kD;
    config.Slot0.kS = Constants.Climb.kS;
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.getConfigurator().apply(config);
  }
  
  /**
   * Sets zero position of relative encoder to position of absolute encoder
   */
  public void home() {
    motor.getConfigurator().setPosition(absEncoder.getAbsPosition());
  }

  /**
   * Stops motor movement
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Changes safeToClimb field value to true
   */
  public void safeToClimb(){
    safeToClimb = true;
  }

  /**
   * Moves robot to new position
   * @param revolutions - number of revolutions to move
   */
  public void moveTo(double revolutions) {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    motor.setControl(request.withPosition(revolutions));
  }
  
  /**
   * If in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * <p> If winding, encoder position must be above latch to run
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if (safeToClimb){
      if (speed.getAsDouble() < 0 && absEncoder.getAbsPosition() > Constants.Climb.LOW_WINCH_ROTATIONS){
        motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
      } else {
        motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
      }
    }
  }

  /**
   * Displays boolean for mode status and values of relative and absolute encoders on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Relative Encoder", () -> motor.getPosition().getValueAsDouble());
    tab.addDouble("Absolute Encoder", () -> absEncoder.getAbsPosition());
    tab.addBoolean("Safe to Climb?", () -> safeToClimb);
  }

  @Override
  public void periodic() {}
}