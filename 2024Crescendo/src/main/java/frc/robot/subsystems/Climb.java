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

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private TalonFX motor;
  private boolean manualMode = false;
  private TalonFXConfiguration config;
  private MotionMagicVoltage request;

  /**
   * Constructor for Climb class
   * <p> Initializing and configuring motor for Motion Magic
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public Climb(int motorID) {
    motor = new TalonFX(motorID);
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
   * Sets zero position of encoder to just above the latch position of the telescoping arm
   */
  public void homeAtTop() {
    motor.getConfigurator().setPosition(Constants.Climb.TOP_ROTATIONS);
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
   * If in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * <p> If winding, encoder position must be above "safe" position to run
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if (manualMode){
      if (speed.getAsDouble() < 0 && motor.getRotorPosition().getValueAsDouble() > Constants.Climb.SAFE_ROTATIONS){
        motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
      } else {
        motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
      }
    }
  }

  /**
   * @param desiredPos - in rotations
   * @return true if the trapezoid profile reaches goal within error bound
   */
  public boolean isReached(double desiredPos){
    return motor.getPosition().getValueAsDouble() > (desiredPos - Constants.Climb.ERROR_ROTATIONS)
      && motor.getPosition().getValueAsDouble() < (desiredPos + Constants.Climb.ERROR_ROTATIONS);
  }

  /**
   * Changes manualMode field value to opposite of current value
   */
  public void switchMode() {
    manualMode = !manualMode;
  }

  /**
   * Stops motor movement
   */
  public void stop() {
    motor.stopMotor();
  }

  public void maintainPos(){}

  /**
   * Displays boolean for mode status and values of relative and absolute encoders on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Relative Encoder", () -> motor.getPosition().getValueAsDouble());
    tab.addBoolean("In Manual Mode?", () -> manualMode);
  }
  
  @Override
  public void periodic() {}
}