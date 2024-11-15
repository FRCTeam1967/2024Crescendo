// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private TalonFX rotateMotor, topMotor, bottomMotor;

  public Turret() {
    rotateMotor = new TalonFX(Constants.Turret.ROTATE_MOTOR_ID);
    topMotor = new TalonFX(Constants.Turret.TOP_MOTOR_ID);
    bottomMotor = new TalonFX(Constants.Turret.BOTTOM_MOTOR_ID);

    //config the shooter
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = Constants.Turret.kP;
    config.Slot0.kI = Constants.Turret.kI;
    config.Slot0.kD = Constants.Turret.kD;
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kA = Constants.Turret.kA;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rotateMotor.getConfigurator().apply(config);
    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);
  }

  public void runShooter(double topVelocity, double topAcceleration, double bottomVelocity, double bottomAcceleration) {
    topMotor.setControl(new VelocityVoltage(topVelocity, topAcceleration, false, 0.0, 0, false, false, false));
    bottomMotor.setControl(new VelocityVoltage(-bottomVelocity, -bottomAcceleration, false, 0.0, 0, false, false, false));
  }

  public void runAlignment(double velocity, double acceleration){
    rotateMotor.setControl(new VelocityVoltage(-velocity, -acceleration, false, 0.0, 0, false, false, false));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
