// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX topMotor, bottomMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new WPI_TalonSRX(Constants.Shooter.TOP_MOTOR_ID);
    bottomMotor = new WPI_TalonSRX(Constants.Shooter.BOTTOM_MOTOR_ID);
  }

  public void runMotors(double speed){
    topMotor.set(speed);
    bottomMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
