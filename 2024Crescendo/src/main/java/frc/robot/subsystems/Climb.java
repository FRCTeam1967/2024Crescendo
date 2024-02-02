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

  private boolean manualMode = false;

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(
      Constants.Climb.MAX_VELOCITY, Constants.Climb.MAX_ACCELERATION);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State(), setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);

  /**
   * Constructor for Climb class
   * <p> Initializing and configuring motors for climb
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public Climb(int motorID, int encoderID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    absEncoder = new Canandcoder(encoderID);
    relEncoder = motor.getEncoder();
    PIDController = motor.getPIDController();
    PIDController.setFeedbackDevice(relEncoder);
    
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Set PID values based on whether supporting robot weight or not
   * @param holdingRobot - true if need to support robot
   */
  public void configurePID(boolean holdingRobot){
    if(holdingRobot){
      PIDController.setP(Constants.Climb.DOWN_kP);
      PIDController.setI(Constants.Climb.DOWN_kI);
      PIDController.setD(Constants.Climb.DOWN_kD);
      PIDController.setFF(Constants.Climb.DOWN_kF);
    } else {
      PIDController.setP(Constants.Climb.UP_kP);
      PIDController.setI(Constants.Climb.UP_kI);
      PIDController.setD(Constants.Climb.UP_kD);
    }
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
   * @param height - desired extension height in meters
   */
  public void moveTo(double height) {
    double revolutions = (height*Constants.Climb.GEAR_RATIO)/(Constants.Climb.SHAFT_DIAMETER*Math.PI);
    goal = new TrapezoidProfile.State(revolutions, 0);
  }

  /**
   * @return true if the trapezoid profile reaches its goal
   */
  public boolean isReached() {
    return (profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  /**
   * Changes manualMode field value to opposite of current value
   */
  public void switchMode() {
    manualMode = !manualMode;
  }

  /**
   * If in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * <p> If winding, encoder position must be above latch to run
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if (manualMode){
      if (speed.getAsDouble() < 0 && absEncoder.getAbsPosition() > Constants.Climb.LOW_HEIGHT) {
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
    tab.addDouble("Relative Encoder", () -> relEncoder.getPosition());
    tab.addDouble("Absolute Encoder", () -> absEncoder.getAbsPosition());
    tab.addBoolean("In Manual Mode?", () -> manualMode);
  }

 @Override
  public void periodic() {
    if(!manualMode) {
      setpoint = profile.calculate(Constants.Climb.UP_kD_TIME, setpoint, goal);
      PIDController.setReference((setpoint.position) * Constants.Climb.GEAR_RATIO, CANSparkBase.ControlType.kPosition);
    }
  }
}