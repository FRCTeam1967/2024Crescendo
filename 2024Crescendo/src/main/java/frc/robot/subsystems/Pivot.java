// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(1.00,0.55);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private SparkPIDController pidController;
  public double revsToMove;
  private RelativeEncoder relativeEncoder;

  private Canandcoder absEncoder;

  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new CANSparkMax(Constants.Pivot.PIVOT_ID, MotorType.kBrushless);

    pidController = pivotMotor.getPIDController();
    pidController.setP(Constants.Pivot.kP);
    pidController.setI(Constants.Pivot.kI);
    pidController.setD(Constants.Pivot.kD);
    pidController.setOutputRange(-0.2, 0.2);

    relativeEncoder = pivotMotor.getEncoder();
    
    absEncoder = new Canandcoder(Constants.Pivot.ENCODER_ID);
    pidController.setFeedbackDevice(relativeEncoder);
  }

  public void pivotHoming(){
    double absAngle = absEncoder.getAbsPosition(); //degrees of revolution (absolute)
    relativeEncoder.setPosition(absAngle);
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  public double getAbsPos() {
    return absEncoder.getAbsPosition();
  }

  public double getRelPos() {
    return relativeEncoder.getPosition();
  }

  public void moveTo(double revolutions) {
    goal = new TrapezoidProfile.State(revolutions, 0);
  }

  public boolean isReached(){
    return(profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
    double revs = (setpoint.position) * Constants.Pivot.GEAR_RATIO;
    pidController.setReference(revs, CANSparkBase.ControlType.kPosition);

    SmartDashboard.putNumber("Rel Pos", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Abs Encoder", absEncoder.getAbsPosition());
    // This method will be called once per scheduler run
  }
}
