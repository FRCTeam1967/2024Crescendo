// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
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

public class TelescopingArm extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder relEncoder;
  private Canandcoder absEncoder;

  private double factor;
  private GenericEntry factorEntry;

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(
      Constants.TelescopingArm.MAX_VELOCITY, Constants.TelescopingArm.MAX_ACCELERATION);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public SparkPIDController PIDController;

  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);

  /**
   * Constructor for TelescopingArm class
   * <p>
   * Initializing and configuring motors for telescoping arms
   * <p>
   * Initializing moveWinch factor for changing speed of arm
   */
  public TelescopingArm(int motorID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);

    absEncoder = new Canandcoder(Constants.TelescopingArm.ENCODER_ID);
    relEncoder = motor.getEncoder();

    PIDController = motor.getPIDController();
    PIDController.setFeedbackDevice(relEncoder);

    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);

    PIDController.setP(Constants.TelescopingArm.kP);
    PIDController.setI(Constants.TelescopingArm.kI);
    PIDController.setD(Constants.TelescopingArm.kD);
    PIDController.setOutputRange(Constants.TelescopingArm.MIN_OUTPUT_RANGE, Constants.TelescopingArm.MAX_OUTPUT_RANGE);

    factor = 0.0;
  }

  public void home() {
    relEncoder.setPosition(absEncoder.getAbsPosition());
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getRelPos() {
    return relEncoder.getPosition();
  }

  public void moveTo(double revolutions) {
    goal = new TrapezoidProfile.State(revolutions, 0);
  }

  public boolean isReached() {
    return (profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  /**
   * Sets speed of leftMotor and rightMotor to speeds of inputs multiplied by factor
   * @param leftSpeed  - left motor speed
   * @param rightSpeed - right motor speed
   */
  public void moveWinch(DoubleSupplier speed) {
    // TODO: replace factorEntry.getDouble() with factorEntry with factor field after tuning
    motor.set(speed.getAsDouble() * factorEntry.getDouble(factor));
  }

  /**
   * Change factor for winch speed
   * 
   * @param newFactor - new value to mutiply to speed for different points in match
   */
  public void changeFactor(double newFactor) {
    factor = newFactor;
  }

  /**
   * Displays current winch factor and boolean showing if climb is winding on
   * Shuffleboard
   * 
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    // tab.addDouble("Current Winch Factor", () -> factor);
    tab.addBoolean("Is Winch Winding?", () -> (factor == Constants.TelescopingArm.WIND_FACTOR));
    factorEntry = tab.add("Winch Factor", factor).getEntry();

    tab.addDouble("Relative Encoder", () -> relEncoder.getPosition());
    tab.addDouble("Absolute Encoder", () -> absEncoder.getAbsPosition());
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.TelescopingArm.kD_TIME, setpoint, goal);
    PIDController.setReference((setpoint.position) * Constants.TelescopingArm.CLIMB_GEAR_RATIO,CANSparkBase.ControlType.kPosition);
  }
}