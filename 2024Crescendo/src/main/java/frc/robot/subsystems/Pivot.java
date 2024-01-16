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

import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.reduxrobotics.canand.CanandDevice;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private CANSparkMax pivotMotor;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(1.00,0.55);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


  public SparkPIDController pidController;
  public double revsToMove;
  private RelativeEncoder relativeEncoder;

  //private SparkAbsoluteEncoder analogEncoder;

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
    
    absEncoder = new Canandcoder(10);
    //analogEncoder = pivotMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    pidController.setFeedbackDevice(relativeEncoder);

    absEncoder.zeroAll();
    //initEncoder();
  }

  public void pivotHoming(){
    double absAngle = absEncoder.getPosition(); //degrees of revolution (absolute)
    relativeEncoder.setPosition(absAngle);
  }

  public void stop() {
    pivotMotor.stopMotor();
  }

  public void limitPower(){
    pivotMotor.setVoltage(0);
  }

  public double getRelPos() {
    return relativeEncoder.getPosition();
  }

  /*public void initEncoder(){
    CANcoderConfiguration config = new CANcoderConfiguration();
    var cancoderConfig = absEncoder.getConfigurator();

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    config.withMagnetSensor(magnetSensorConfigs);
    magnetSensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    //magnetSensorConfigs.MagnetOffset = Constants.Pivot.OFFSET;
    cancoderConfig.apply(magnetSensorConfigs);
  }*/

  public void moveTo(double revolutions) {
    double currentAbsRevPos = absEncoder.getPosition();
    double revsToMove = revolutions - currentAbsRevPos;
    goal = new TrapezoidProfile.State(revsToMove, 0);
  }

  public boolean isReached(){
    return(profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  @Override
  public void periodic() {
    if (setpoint.position != goal.position){
      setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
      double revs = (setpoint.position) * Constants.Pivot.GEAR_RATIO;
      pidController.setReference(revs, CANSparkBase.ControlType.kPosition);
    } 
    SmartDashboard.putNumber("Rel Pos", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Abs Encoder", absEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
