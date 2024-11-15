// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class TurretAlign extends Command {
  private final Turret turret;
  private double goal;

  public TurretAlign(Turret turret, double goal) {
    this.turret = turret;
    this.goal = goal;
    
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.runAlignment(Constants.Turret.TURRET_VELOCITY, Constants.Turret.TURRET_ACCELERATION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("aligned", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
