// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    topLeftMotor = new TalonFX(Constants.Shooter.TOP_LEFT_MOTOR_ID);
    topRightMotor = new TalonFX(Constants.Shooter.TOP_RIGHT_MOTOR_ID);
    bottomLeftMotor = new TalonFX(Constants.Shooter.BOTTOM_LEFT_MOTOR_ID);
    bottomRightMotor = new TalonFX(Constants.Shooter.BOTTOM_RIGHT_MOTOR_ID);

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

    topLeftMotor.getConfigurator().apply(config);
    topRightMotor.getConfigurator().apply(config);
    bottomLeftMotor.getConfigurator().apply(config);
    bottomRightMotor.getConfigurator().apply(config);
  }

   public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        String name = getName();
        builder.addDoubleProperty("topLeftMotorVelocity",()->getMotorVelocity(topLeftMotor),null);
        builder.addDoubleProperty("bottomLeftMotorVelocity",()->getMotorVelocity(bottomLeftMotor),null);
        builder.addDoubleProperty("topRightMotorVelocity",()->getMotorVelocity(topRightMotor),null);
        builder.addDoubleProperty("bottomRightMotorVelocity",()->getMotorVelocity(bottomRightMotor),null);
    }

  /**
   * Run all shooter motors at inputted velocities/accelerations
   * @param topSpeed - percentage of full power (1.0)
   * @param bottomSpeed - percentage of full power (1.0)
   */
  public void runNoPID(double topSpeed, double bottomSpeed) {
    topLeftMotor.set(-topSpeed);
    topRightMotor.set(topSpeed);
    bottomLeftMotor.set(-bottomSpeed);
    bottomRightMotor.set(bottomSpeed);
  }

  public void runShooter(double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    topLeftMotor.setControl(new VelocityVoltage(-topVelocity, -topAcceleration, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
    bottomLeftMotor.setControl(new VelocityVoltage(-bottomVelocity, -bottomAcceleration, false, 0.0, 0, false, false, false));
    bottomRightMotor.setControl(new VelocityVoltage(bottomVelocity, bottomAcceleration, false, 0.0, 0, false, false, false));
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
    topLeftMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    topRightMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomLeftMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
    bottomRightMotor.setControl(new VelocityVoltage(0, 0, false, 0.0, 0, false, false, false));
  }

  public void configDashboard(ShuffleboardTab tab) {
    // tab.addDouble("Average Velocity", ()->getAverageVelocity());
    // tab.addDouble("Top Left", ()->getMotorVelocity(topLeftMotor));
    // tab.addDouble("Top Right", ()->getMotorVelocity(topRightMotor));
    // tab.addDouble("Bottom Left", ()->getMotorVelocity(bottomLeftMotor));
    // tab.addDouble("Bottom Right", ()->getMotorVelocity(bottomRightMotor));
  }

  public double getAverageVelocity(){
    return (getMotorVelocity(topLeftMotor) + getMotorVelocity(topRightMotor) + getMotorVelocity(bottomLeftMotor) + getMotorVelocity(bottomRightMotor))/4.0;
  }

  public double getAverageTopVelocity(){
    return (getMotorVelocity(topLeftMotor) + getMotorVelocity(topRightMotor))/2.0;
  }

  public double getMotorVelocity(TalonFX motor){
    return Math.abs(motor.getVelocity().getValueAsDouble()); 
  }

  @Override
  public void periodic() {}
}