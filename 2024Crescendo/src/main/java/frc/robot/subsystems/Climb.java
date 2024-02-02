// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkBase.*;
import com.revrobotics.CANSparkBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder relEncoder;
  private Canandcoder absEncoder;
  public SparkPIDController PIDController;

  private boolean manualMode = true;

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(
      Constants.Climb.MAX_VELOCITY, Constants.Climb.MAX_ACCELERATION);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State(), setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);

  /**
   * Constructor for Climb class
   * <p>
   * Initializing and configuring motors for climb
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public Climb(int motorID, int encoderID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    absEncoder = new Canandcoder(encoderID);
    relEncoder = motor.getEncoder();
    PIDController = motor.getPIDController();
    
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);

    PIDController.setFeedbackDevice(relEncoder);
    PIDController.setP(Constants.Climb.kP);
    PIDController.setI(Constants.Climb.kI);
    PIDController.setD(Constants.Climb.kD);
    PIDController.setOutputRange(Constants.Climb.MIN_OUTPUT_RANGE, Constants.Climb.MAX_OUTPUT_RANGE);
  }
  
  /**
   * Sets zero position of relative encoder to position of absolute encoder
   */
  public void home() {
    relEncoder.setPosition(absEncoder.getAbsPosition());
  }

  /**
   * Stops motor movement
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * @return relative encoder's position
   */
  public double getRelPos() {
    return relEncoder.getPosition();
  }
  
  /**
   * Assigns new position for robot to move to based on current state
   * @param revolutions - number of revolutions to move
   */
  public void moveTo(double revolutions) {
    goal = new TrapezoidProfile.State(revolutions, 0);
  }

  /**
   * @return true if the trapezoid profile reaches its goal
   */
  public boolean isReached() {
    return (profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  /**
   * Changes boolean field to manual or not manual mode
   */
  public void changeMode() {
    manualMode = !manualMode;
  }

  /**
   * Sets unwind/wind factor dependent on pos/neg value of speed
   * <p> If in manual mode, runs motor
   * @param speed - speed of motor
   */
  public void moveWinch(DoubleSupplier speed) {
    if (manualMode){
      if (speed.getAsDouble() > 0) {
        motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
      } else {
        motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
      }
    }
  }

  /**
   * Displays boolean for mode status and values of relative and absolute encoders on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    // tab.addDouble("Relative Encoder", () -> relEncoder.getPosition());
    // tab.addDouble("Absolute Encoder", () -> absEncoder.getAbsPosition());
    tab.addBoolean("Manual Mode?", () -> manualMode);
  }

 @Override
  public void periodic() {
    if(!manualMode) {
      setpoint = profile.calculate(Constants.Climb.kD_TIME, setpoint, goal);
      PIDController.setReference((setpoint.position) * Constants.Climb.CLIMB_GEAR_RATIO, CANSparkBase.ControlType.kPosition);
    }
  }
}