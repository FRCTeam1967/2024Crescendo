// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class MoveAmpBar extends Command {
  private AmpBar ampBar;
  private double desiredRev;

  /**
   * Creates a new MoveAmpBar object
   * @param ampbar - AmpBar object
   * @param rev - revolutions for desired position
   */
  public MoveAmpBar(AmpBar ampBar, double rev) {
    this.ampBar = ampBar; 
    desiredRev = rev;
    addRequirements(this.ampBar);
  }

  @Override
  public void initialize() {
    ampBar.moveTo(desiredRev);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    ampBar.stop();
  }

  @Override
  public boolean isFinished() {
    return ampBar.isReached();
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // String name = getName();
    builder.addDoubleProperty("target pos", () -> {return desiredRev;}, (var) -> {desiredRev = var;});
  }
}