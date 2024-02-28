// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RunPivotIntakeBeam extends Command {
  private Feeder feeder;
  private Intake intake;
  private Pivot pivot;
  private double leftSpeed, rightSpeed;
  
  /**
   * Creates a new RunFeeder
   * @param feeder - Feeder object
   * @param leftSpeed
   * @param rightSpeed
   */
  public RunPivotIntakeBeam(Pivot pivot, Intake intake, Feeder feeder, double leftSpeed, double rightSpeed) {
    this.intake = intake;
    this.feeder = feeder;
    this.pivot = pivot;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(this.pivot, this.intake, this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivot.moveTo(Constants.Pivot.INTAKE_DOWN);
    intake.runMotors(Constants.Intake.INTAKE_ROLLER_SPEED);
    feeder.feedFeeder(leftSpeed, rightSpeed);
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
