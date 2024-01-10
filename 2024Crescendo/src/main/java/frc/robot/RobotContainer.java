// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;


public class RobotContainer {
  
  public final Swerve swerve = new Swerve();

  private final Command turnToAngle = new RunCommand(() -> {
    swerve.goToAngle(100);
  }, swerve);
  
  private final Command goToDefenseMode = new RunCommand(() -> {
    swerve.defenseMode();
  }, swerve);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    //Testing if it will move to specific angle
    m_driverController.button(4).onTrue(turnToAngle);
    m_driverController.button(2).onTrue(goToDefenseMode);

    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -m_driverController.getRawAxis(1),
      () -> m_driverController.getRawAxis(0), () -> -m_driverController.getRawAxis(4)));
    
  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("Score Middle");

  }
}
