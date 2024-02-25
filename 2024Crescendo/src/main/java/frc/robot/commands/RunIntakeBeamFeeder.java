// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class RunIntakeBeamFeeder extends Command {
  private Feeder feeder;
  private Intake intake;
  private double leftSpeed, rightSpeed;
  
  /**
   * Creates a new RunFeeder
   * @param feeder - Feeder object
   * @param leftSpeed
   * @param rightSpeed
   */
  public RunIntakeBeamFeeder(Intake intake, Feeder feeder, double leftSpeed, double rightSpeed) {
    this.intake = intake;
    this.feeder = feeder;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    while(!feeder.isBroken()){
      intake.runMotors(Constants.Intake.INTAKE_ROLLER_SPEED);
      feeder.feedFeeder(leftSpeed, rightSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return feeder.isBroken();
  }
}
