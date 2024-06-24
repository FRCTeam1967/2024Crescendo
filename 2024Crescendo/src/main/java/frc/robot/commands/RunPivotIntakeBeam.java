// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class RunPivotIntakeBeam extends Command {
  private Feeder feeder;
  private Intake intake;
  private Pivot pivot;
  
  /**
   * Creates a new RunPivotIntakeBeam
   * @param pivot - Pivot object
   * @param intake - Intake object
   * @param feeder - Feeder object
   */
  public RunPivotIntakeBeam(Pivot pivot, Intake intake, Feeder feeder) {
    this.intake = intake;
    this.feeder = feeder;
    this.pivot = pivot;
    addRequirements(this.pivot, this.intake, this.feeder);
  }

  @Override
  public void initialize() {
    pivot.moveTo(Constants.Pivot.INTAKE_DOWN);

  }

  @Override
  public void execute() {
    intake.runMotors(Constants.Intake.INTAKE_ROLLER_SPEED);
    feeder.feedFeeder(Constants.Feeder.FEED_SPEED);
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
