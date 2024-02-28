// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private Canandcoder absEncoder;
  private SparkPIDController pidController;
  private RelativeEncoder relativeEncoder;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(12,8);
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  
  public double revsToMove;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new CANSparkMax(Constants.Pivot.PIVOT_ID, MotorType.kBrushless);
    pivotMotor.setInverted(true);    
    pidController = pivotMotor.getPIDController();
    pidController.setP(Constants.Pivot.kP);
    pidController.setI(Constants.Pivot.kI);
    pidController.setD(Constants.Pivot.kD);
    pidController.setOutputRange(-0.2, 0.2);

    relativeEncoder = pivotMotor.getEncoder();
    
    absEncoder = new Canandcoder(Constants.Pivot.ENCODER_ID);
    pidController.setFeedbackDevice(relativeEncoder);

    Canandcoder.Settings settings = new Canandcoder.Settings();
    settings.setInvertDirection(true);
    absEncoder.setSettings(settings, 0.050);
  }

  /** Sets relative encoder value to absolute encoder value */
  public void setRelToAbs(){
    REVLibError success = relativeEncoder.setPosition(absEncoder.getAbsPosition()*Constants.Pivot.GEAR_RATIO);
    System.out.println("REV error" + success); 
  }

  /** Stops pivot motor */
  public void stop() {
    pivotMotor.stopMotor();
  }

  /**
   * @return position from absolute encoder
   */
  public double getAbsPos() {
    return absEncoder.getAbsPosition();
  }

  /**
   * Sets motion profiling goal to desired revolutions
   * @param revolutions
   */
  public void moveTo(double revolutions) {
    goal = new TrapezoidProfile.State(revolutions, 0);
  }

  /**
   * @return whether profile has been finished
   */
  public boolean isReached(){
    return(profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  /** Sets pivot motor to brake mode */
  public void setBrakeMode(){
    pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
    double revs = (setpoint.position) * Constants.Pivot.GEAR_RATIO;
    pidController.setReference(revs, CANSparkBase.ControlType.kPosition);

    SmartDashboard.putNumber("Rel Pos", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Abs Encoder", absEncoder.getAbsPosition());
    SmartDashboard.putNumber("Set Point", setpoint.position); 
    SmartDashboard.putNumber("revs", revs); 
    SmartDashboard.putNumber("Rel Pos Degrees", (relativeEncoder.getPosition()*360)/50);
    SmartDashboard.putNumber("Abs Encoder Degrees", absEncoder.getAbsPosition()*360);
  }
}