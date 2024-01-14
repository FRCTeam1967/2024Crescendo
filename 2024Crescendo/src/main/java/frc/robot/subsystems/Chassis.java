// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants;

public class Chassis extends SubsystemBase {
  private WPI_TalonSRX leftLeader, leftFollower, rightLeader, rightFollower;
  private MotorControllerGroup leftMotorControllerGroup, rightMotorControllerGroup;
  private DifferentialDrive differentialDrive;

  /** Creates a new Chassis. */
  public Chassis() {
    leftLeader = new WPI_TalonSRX(Constants.Chassis.LEFT_LEADER_ID);
    leftFollower = new WPI_TalonSRX(Constants.Chassis.LEFT_FOLLOWER_ID);
    rightLeader = new WPI_TalonSRX(Constants.Chassis.RIGHT_LEADER_ID);
    rightFollower = new WPI_TalonSRX(Constants.Chassis.RIGHT_FOLLOWER_ID);

    leftLeader.setNeutralMode(NeutralMode.Coast);
    leftFollower.setNeutralMode(NeutralMode.Coast);
    rightLeader.setNeutralMode(NeutralMode.Coast);
    rightFollower.setNeutralMode(NeutralMode.Coast);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    
    leftMotorControllerGroup = new MotorControllerGroup(leftLeader, leftFollower);
    rightMotorControllerGroup = new MotorControllerGroup(rightLeader, rightFollower);

    differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);
  }

  public void drive(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick){
    differentialDrive.tankDrive(Math.pow(leftJoystick.getAsDouble(), Constants.Chassis.JOYSTICK_EXP), Math.pow(rightJoystick.getAsDouble(),Constants.Chassis.JOYSTICK_EXP));
  }

  public void driveStraight(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick){
    double joystickAvg = (leftJoystick.getAsDouble() + rightJoystick.getAsDouble())/2;
    differentialDrive.tankDrive(Math.pow(joystickAvg, Constants.Chassis.JOYSTICK_EXP), Math.pow(joystickAvg,Constants.Chassis.JOYSTICK_EXP));
  }

  public void driveSlow(DoubleSupplier leftJoystick, DoubleSupplier rightJoystick){
    double joystickAvg = (leftJoystick.getAsDouble() + rightJoystick.getAsDouble())/2;
    differentialDrive.tankDrive(Math.pow((joystickAvg*Constants.Chassis.SLOW_MODE_FACTOR), Constants.Chassis.JOYSTICK_EXP), Math.pow((joystickAvg*Constants.Chassis.SLOW_MODE_FACTOR), Constants.Chassis.JOYSTICK_EXP));
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
