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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import frc.robot.Constants;

public class AmpBar extends SubsystemBase {
  private CANSparkMax ampBarMotor;
  private SparkPIDController pidController;
  private RelativeEncoder relativeEncoder;
  private Timer timer;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(Constants.AmpBar.MAX_VELOCITY, Constants.AmpBar.MAX_ACCELERATION);
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  
  public double revsToMove;

  /** Creates a new Pivot. */
  public AmpBar() {
    ampBarMotor = new CANSparkMax(Constants.AmpBar.AMP_BAR_ID, MotorType.kBrushless);
    ampBarMotor.setInverted(true);    
    pidController = ampBarMotor.getPIDController();
    pidController.setP(Constants.AmpBar.kP);
    pidController.setI(Constants.AmpBar.kI);
    pidController.setD(Constants.AmpBar.kD);
    pidController.setOutputRange(-0.25, 0.25);

    relativeEncoder = ampBarMotor.getEncoder();
    
    pidController.setFeedbackDevice(relativeEncoder);
    timer = new Timer();
  }

  /** Stops amp bar motor */
  public void stop() {
    ampBarMotor.stopMotor();
  }

  public double getPosition(){
    return relativeEncoder.getPosition();
  }

  public void runSecond(){
    timer.start();
    while (timer.get() <= 2){
      ampBarMotor.set(-0.2);
    }
    ampBarMotor.set(0);
    timer.stop();
    timer.reset();
  }

  /** Set encoder position to desired revolutions
  * @param rev
  */
  public void setPosition(double rev){
    relativeEncoder.setPosition(rev);
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

  /** Sets amp bar motor to brake mode */
  public void setBrakeMode(){
    ampBarMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.AmpBar.kD_TIME, setpoint, goal);
    double revs = (setpoint.position) * Constants.AmpBar.GEAR_RATIO;
    pidController.setReference(revs, CANSparkBase.ControlType.kPosition);

    SmartDashboard.putNumber("Amp Rel Pos", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Amp Set Point", setpoint.position); 
    SmartDashboard.putNumber("Amp revs", revs); 
    SmartDashboard.putNumber("Amp Rel Pos Degrees", (relativeEncoder.getPosition()*360)/100);
  }
}