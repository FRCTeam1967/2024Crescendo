// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;


public class ShootAcrossField extends Command {
    private Shooter shooter;
    private double velocity;
    private Timer timer;
  /** Creates a new ShootAcrossField. */
  public ShootAcrossField(Shooter shooter) {

  
    this.shooter = shooter;
    addRequirements(this.shooter);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   shooter.runTop(Constants.Shooter.FEEDER_SPEED, Constants.Shooter.FEEDER_ACCELERATION);
      if(timer.get()>Constants.Shooter.TIME){
        shooter.runBottom(Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
