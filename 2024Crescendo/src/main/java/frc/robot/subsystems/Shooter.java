// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax topMotor, bottomMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    topMotor = new CANSparkMax(Constants.Shooter.TOP_MOTOR_ID, MotorType.kBrushed);
    bottomMotor = new CANSparkMax(Constants.Shooter.BOTTOM_MOTOR_ID, MotorType.kBrushed);

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

  } 

  /**
   * Run all shooter motors at inputted velocities/accelerations
   * @param topVelocity - velocity for top wheels
   * @param bottomVelocity - velocity for bottom wheels
   */
  public void runShooter(double topVelocity, double bottomVelocity) {
    topMotor.set(topVelocity);
    bottomMotor.set(bottomVelocity);
    
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
  public void stop() {
      topMotor.set(0.0);
      bottomMotor.set(0.0);
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Velocity", ()->Constants.Shooter.SPEAKER_VELOCITY);

  }

  @Override
  public void periodic() {}

}