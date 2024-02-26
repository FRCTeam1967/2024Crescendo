// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.Feeder;

public class RumbleController extends Command {
  /** Creates a new RumbleController. */
  private CommandXboxController driverXbox;
  private CommandXboxController operatorXbox;
  private Feeder feeder;
  public RumbleController(CommandXboxController driverXbox, CommandXboxController operatorXbox,Feeder feeder) {
    this.driverXbox = driverXbox;
    this.operatorXbox = operatorXbox;
    this.feeder = feeder;
    addRequirements(feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.8);
    operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0.8);
  }

  // Called once the command ends or is interrupted.
  @Override 
  public void end(boolean interrupted) {
    driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
    operatorXbox.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return feeder.isBroken();
    return false;
  }
}
