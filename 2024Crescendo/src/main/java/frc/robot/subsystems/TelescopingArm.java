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

public class TelescopingArm extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder relEncoder;
  private Canandcoder absEncoder;
  public SparkPIDController PIDController;

  private double factor;

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(
      Constants.TelescopingArm.MAX_VELOCITY, Constants.TelescopingArm.MAX_ACCELERATION);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);

  /**
   * Constructor for TelescopingArm class
   * <p>
   * Initializing and configuring motors for telescoping arms
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
   * Sets speed of motor to input multiplied by factor
   * @param speed - speed of motor needed in order to
   */
  public void moveWinch(DoubleSupplier speed) {
    if(speed.getAsDouble()>0) factor = Constants.TelescopingArm.UNWIND_FACTOR;
    else factor = Constants.TelescopingArm.WIND_FACTOR;

    motor.set(speed.getAsDouble() * factor);
  }

  /**
   * Displays current winch factor, takes in entered factor, and boolean showing if climb is winding on Shuffleboard
   * <p> Also displays values of relaive and absolute encoders
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    tab.addBoolean("Is Winch Winding?", () -> (factor == Constants.TelescopingArm.WIND_FACTOR));

    tab.addDouble("Relative Encoder", () -> relEncoder.getPosition());
    tab.addDouble("Absolute Encoder", () -> absEncoder.getAbsPosition());
  }

 @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.TelescopingArm.kD_TIME, setpoint, goal);
    PIDController.setReference((setpoint.position) * Constants.TelescopingArm.CLIMB_GEAR_RATIO,CANSparkBase.ControlType.kPosition);
  }
}