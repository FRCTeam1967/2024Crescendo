// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  private final Shooter shooter = new Shooter();

  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");


  public RobotContainer() {

    CanandEventLoop.getInstance();

    configureBindings();

    shooter.configDashboard(matchTab);
  }

  
  private void configureBindings() {
    //DEFAULT COMMANDS
    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new RunShooter(shooter, Constants.Shooter.SPEAKER_VELOCITY));
    operatorController.leftBumper().or(operatorController.rightBumper()).whileTrue(new RunShooter(shooter, -Constants.Shooter.SPEAKER_VELOCITY));
    //shooter.setDefaultCommand(new InstantCommand(() -> shooter.stopMotors()));
  }
}