// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX topMotor, bottomMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
    bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = Constants.Shooter.kP;
    config.Slot0.kI = Constants.Shooter.kI;
    config.Slot0.kD = Constants.Shooter.kD;
    config.Slot0.kV = Constants.Shooter.kV;
    config.Slot0.kA = Constants.Shooter.kA;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);
  }

  /**
   * Run all shooter motors at inputted velocities/accelerations
   * @param topSpeed - percentage of full power (1.0)
   * @param bottomSpeed - percentage of full power (1.0)
   */
  public void runNoPID(double topSpeed, double bottomSpeed) {
    topMotor.set(topSpeed);
    bottomMotor.set(topSpeed);
  }

  public void runShooter(double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    topMotor.setControl(new VelocityVoltage(-topVelocity, -topAcceleration, false, 0.0, 0, false, false, false));
    bottomMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
  }

  /*public void runTopPID(double topVelocity, double topAcceleration, double bottomSpeed) {
    topLeftMotor.setControl(new VelocityVoltage(-topVelocity, -topAcceleration, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
    bottomLeftMotor.set(-bottomSpeed);
    bottomRightMotor.set(bottomSpeed);
  }*/

  /**
   * Stops all shooter motors
   */
  public void stopMotors() {
    topMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Average Velocity", ()->getAverageVelocity());
    tab.addDouble("Top Left", ()->getMotorVelocity(topMotor));
    tab.addDouble("Top Right", ()->getMotorVelocity(bottomMotor));
  }

  public double getAverageVelocity(){
    return (getMotorVelocity(topMotor) + getMotorVelocity(bottomMotor))/4.0;
  }

  public double getTopVelocity(){
    return getMotorVelocity(topMotor);
  }

  public double getMotorVelocity(TalonFX motor){
    return Math.abs(motor.getVelocity().getValueAsDouble()); 
  }

  @Override
  public void periodic() {}
}