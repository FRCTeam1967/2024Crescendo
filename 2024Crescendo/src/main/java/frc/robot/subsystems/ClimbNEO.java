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

import java.util.function.DoubleSupplier;

public class ClimbNEO extends SubsystemBase implements Climb {
  private CANSparkMax motor;
  private RelativeEncoder relEncoder;
  private SparkPIDController PIDController;

  private boolean manualMode = false;

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(
      Constants.Climb.MAX_VELOCITY, Constants.Climb.MAX_ACCELERATION);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State(), setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);

  /**
   * Constructor for ClimbNEO class
   * <p> Initializing and configuring motors for climb
   * @param motorID - port for motor
   * @param encoderID - port for absolute encoder
   */
  public ClimbNEO(int motorID) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    relEncoder = motor.getEncoder();
    PIDController = motor.getPIDController();
    PIDController.setFeedbackDevice(relEncoder);

    PIDController.setOutputRange(Constants.Climb.MIN_OUTPUT_RANGE, Constants.Climb.MAX_OUTPUT_RANGE);
    
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
  }
  
  /**
   * Tell relative encoder that it's at the top position
   */
  public void homeAtTop() {
    relEncoder.setPosition(Constants.Climb.TOP_ROTATIONS);
  }
  
  /**
   * Assigns new position for robot to move to based on current state
   * @param pos - desired position to go to in rotations
   * @param holdingRobot - whether holding robot weight, used to configure PID accordingly
   */
  public void moveTo(double pos, boolean holdingRobot) {
    if(holdingRobot){ //configure PID accordingly
      PIDController.setP(Constants.Climb.DOWN_kP);
      PIDController.setI(Constants.Climb.DOWN_kI);
      PIDController.setD(Constants.Climb.DOWN_kD);
      PIDController.setFF(Constants.Climb.DOWN_kF);
    } else {
      PIDController.setP(Constants.Climb.UP_kP);
      PIDController.setI(Constants.Climb.UP_kI);
      PIDController.setD(Constants.Climb.UP_kD);
    }
    goal = new TrapezoidProfile.State(pos, 0);
  }

  /**
   * If in manual mode, sets unwind/wind factor dependent on pos/neg value of speed and runs motor
   * <p> If winding, encoder position must be above latch to run
   * @param speed - speed of motor
   */
  public void moveAt(DoubleSupplier speed) {
    if (manualMode){
      if (speed.getAsDouble() < 0 && relEncoder.getPosition() > Constants.Climb.SAFE_ROTATIONS) {
        motor.set(speed.getAsDouble() * Constants.Climb.WIND_FACTOR);
      } else {
        motor.set(speed.getAsDouble() * Constants.Climb.UNWIND_FACTOR);
      }
    }
  }

  /**
   * @param desiredPos - not doing anything for NEO
   * @return true if the trapezoid profile reaches its goal
   */
  public boolean isReached(double desiredPos) {
    return (profile.isFinished(profile.timeLeftUntil(goal.position)));
  }

  /**
   * Changes manualMode field value to opposite of current value
   */
  public void switchMode() {
    manualMode = !manualMode;
  }

  /**
   * Stops motor movement
   */
  public void stop() {
    motor.stopMotor();
  }

  /**
   * Establishes starting setpoint and goal for motion profiling, called in teleopInit
   */
  public void maintainPos(){
    setpoint.velocity = 0;
    setpoint.position = relEncoder.getPosition();
    goal.velocity = 0;
    goal.position = relEncoder.getPosition();
  }

  /**
   * Displays boolean for mode status and values of relative and absolute encoders on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Relative Encoder", () -> relEncoder.getPosition());
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