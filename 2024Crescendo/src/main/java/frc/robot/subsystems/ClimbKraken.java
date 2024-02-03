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

public class ClimbKraken extends SubsystemBase {
  private TalonFX motor;
  private Canandcoder absEncoder;
  private boolean manualMode = false;
  private TalonFXConfiguration config;
  private MotionMagicVoltage request;

  /**
   * Constructor for Climb class
   * <p> Initializing and configuring motor for Motion Magic
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public ClimbKraken(int motorID, int encoderID) {
    motor = new TalonFX(motorID);
    absEncoder = new Canandcoder(encoderID);
    config = new TalonFXConfiguration();
    request = new MotionMagicVoltage(0);
    
    motor.setNeutralMode(NeutralModeValue.Brake);

    config.Slot0.kP = Constants.Climb.UP_kP;
    config.Slot0.kI = Constants.Climb.UP_kI;
    config.Slot0.kD = Constants.Climb.UP_kD;

    config.Slot1.kP = Constants.Climb.DOWN_kP;
    config.Slot1.kI = Constants.Climb.DOWN_kI;
    config.Slot1.kD = Constants.Climb.DOWN_kD;
    config.Slot1.kS = Constants.Climb.DOWN_kF;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Climb.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = Constants.Climb.ACCELERATION;
    
    motor.getConfigurator().apply(config);
  }
  
  /**
   * Sets zero position of relative encoder to position of absolute encoder
   */
  public void home() {
    motor.getConfigurator().setPosition(absEncoder.getAbsPosition());
    //assuming under 1 rotation (absolute) -- make sure we're starting down
  }

  /**
   * Stops motor movement
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Changes manualMode field value to opposite of current value
   */
  public void switchMode() {
    manualMode = !manualMode;
  }

  /**
   * Moves robot to new height using Motion Magic
   * @param height - desired extension height
   * @param slot - configuration slot to use (0 for PID values without holding robot weight, 1 for with robot weight)
   */
  public void moveTo(double height, int slot) {
    double revolutions = (height*Constants.Climb.GEAR_RATIO)/(Constants.Climb.SHAFT_DIAMETER*Math.PI);
    motor.setControl(request.withPosition(revolutions).withSlot(slot));
  }
  
  /**
   * If in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * <p> If winding, encoder position must be above latch to run
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if (manualMode){
      if (speed.getAsDouble() < 0 && absEncoder.getAbsPosition() > Constants.Climb.LOW_HEIGHT){
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
    tab.addBoolean("In Manual Mode?", () -> manualMode);
  }

  @Override
  public void periodic() {}
}