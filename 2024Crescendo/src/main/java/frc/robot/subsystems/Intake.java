// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
  private CANSparkMax motor;

  /** Creates new Intake */
  public Intake() {
    motor = new CANSparkMax(Constants.Intake.MOTOR_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
  }

   public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        String name = getName();
        builder.addDoubleProperty("intakeSpeed",()->motor.get(),null);
    }

  /**
   * Sets speed of motor to speed of input
   * @param speed - motor speed
   */
  public void runMotors(double speed){
    motor.set(speed);
  }

  /** Stops intake motor */
  public void stopMotors(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {}
}