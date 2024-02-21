// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class RunIntake extends Command {
  Intake intake;
  double speed;
  
  /**
   * Creates a new RunIntake
   * @param intake - Intake object
   * @param speed - desired speed
   */
  public RunIntake(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.runMotors(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}