// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;



import frc.robot.Constants;
import frc.robot.MathHelper;

public class AmpBar extends SubsystemBase {
  private SparkMax ampBarMotor;
  private SparkClosedLoopController pidController;
  private RelativeEncoder relativeEncoder;
  private Timer timer;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(Constants.AmpBar.MAX_VELOCITY, Constants.AmpBar.MAX_ACCELERATION);
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  public TrapezoidProfile.State goal = new TrapezoidProfile.State();
  
  public double revsToMove;

  /** Creates a new Pivot. */
  public AmpBar() {
    ampBarMotor = new SparkMax(Constants.AmpBar.AMP_BAR_ID, MotorType.kBrushless);

    pidController = ampBarMotor.getClosedLoopController();

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
        .pid(Constants.AmpBar.kP, Constants.AmpBar.kI, Constants.AmpBar.kD)
        .outputRange(-0.25, 0.25);
        
    ampBarMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    relativeEncoder = ampBarMotor.getEncoder();
    
    timer = new Timer();
  }

  /** Stops amp bar motor */
  public void stop() {
    ampBarMotor.stopMotor();
  }

  public double getPosition(){
    return relativeEncoder.getPosition();
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
    return profile.timeLeftUntil(goal.position) <= 0;
  }

  // /** Sets amp bar motor to brake mode */
  // public void setBrakeMode(){
  //   ampBarMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  // }

  // /** Sets amp bar motor to coast mode */
  // public void setCoastMode(){
  //   ampBarMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
  // }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Amp Rel Pos", () -> relativeEncoder.getPosition());
    tab.addDouble("Amp Set Point", () -> setpoint.position);
    tab.addDouble("Amp Rel Pos Degrees", () -> (relativeEncoder.getPosition()*360)/100);
  }

  @Override
  public void periodic() {
    setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
    double revs = (setpoint.position) * Constants.AmpBar.GEAR_RATIO;
    pidController.setReference(revs, SparkBase.ControlType.kPosition);
  }
}

  // public void runSecond(){
  //   timer.start();
  //   while (timer.get() <= 2){
  //     ampBarMotor.set(-0.2);
  //   }
  //   ampBarMotor.set(0);
  //   timer.stop();
  //   timer.reset();
  // }