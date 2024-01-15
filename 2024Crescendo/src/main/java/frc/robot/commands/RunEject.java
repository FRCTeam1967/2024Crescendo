// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants;

public class RunEject extends Command {
  private final Shooter m_shooter;
  private Timer timer;

  public RunEject(Shooter shooter) {
    m_shooter = shooter;
    timer = new Timer();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_shooter.runMotors(Constants.Shooter.SHOOTER_EJECT,0);
    m_shooter.runTop(Constants.Shooter.SHOOTER_EJECT);
    if(timer.get()>Constants.Shooter.TIME){
      m_shooter.runBottom(Constants.Shooter.SHOOTER_EJECT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
