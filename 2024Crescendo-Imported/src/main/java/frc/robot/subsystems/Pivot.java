// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.CanandmagSettings;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private SparkMax pivotMotor;
  private Canandmag absEncoder;
  private SparkClosedLoopController pidController;
  private RelativeEncoder relativeEncoder;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(Constants.Pivot.MAX_VELOCITY, Constants.Pivot.MAX_ACCELERATION);
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  
  public double revsToMove;

  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new SparkMax(Constants.Pivot.PIVOT_ID, MotorType.kBrushless);
    pidController = pivotMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(35);

    config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    config.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.Pivot.kP, Constants.Pivot.kI, Constants.Pivot.kD)
        .outputRange(-0.4, 0.5);
        
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    absEncoder = new Canandmag(Constants.Pivot.ENCODER_ID);
    relativeEncoder = pivotMotor.getEncoder();

    CanandmagSettings settings = new CanandmagSettings();
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
  // public void setBrakeMode(){
  //   pivotMotor.idleMode(SparkBaseConfig.IdleMode.kBrake);
  // }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Rel Pos", () -> relativeEncoder.getPosition());
    tab.addDouble("Abs Encoder", () -> absEncoder.getAbsPosition());
    tab.addDouble("Set Point", () -> setpoint.position); 
    tab.addDouble("Rel Pos Degrees", () -> (relativeEncoder.getPosition()*360)/50);
    tab.addDouble("Abs Encoder Degrees", () -> absEncoder.getAbsPosition()*360);
    tab.addDouble("Pivot Voltage motorcontroller Output Current in Amps", () -> pivotMotor.getOutputCurrent());
    tab.addDouble("Pivot Voltage into motorcontroller", () -> pivotMotor.getBusVoltage());
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
    double revs = (setpoint.position) * Constants.Pivot.GEAR_RATIO;
    pidController.setReference(revs, SparkBase.ControlType.kPosition);
  }
}