// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Chassis m_chassis = new Chassis();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xbox = new CommandXboxController(2);
  
  private CommandJoystick leftJoystick = new CommandJoystick(0);
  private CommandJoystick rightJoystick = new CommandJoystick(1);

  private ShuffleboardTab tab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    tab = Shuffleboard.getTab("tab");
    tab.add(CommandScheduler.getInstance());
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_xbox.leftTrigger().whileTrue(new RunCommand(() -> m_shooter.runMotors(Constants.Shooter.SHOOTER_INTAKE), m_shooter));
    //m_xbox.rightTrigger().whileTrue(new RunCommand(() -> m_shooter.runMotors(Constants.Shooter.SHOOTER_EJECT), m_shooter));
    m_xbox.rightTrigger().whileTrue(new RunEject(m_shooter));
    m_shooter.setDefaultCommand(new RunCommand(()-> m_shooter.runMotors(0.0), m_shooter));

    leftJoystick.button(1).or(rightJoystick.button(1)).whileTrue(new RunCommand(() -> m_chassis.driveStraight(
      () -> MathUtil.applyDeadband(leftJoystick.getY(), Constants.Chassis.JOYSTICK_DEADBAND),
      () -> MathUtil.applyDeadband(rightJoystick.getY(), Constants.Chassis.JOYSTICK_DEADBAND)), m_chassis));
    
    leftJoystick.button(2).or(rightJoystick.button(2)).whileTrue(new RunCommand(() -> m_chassis.driveSlow(
      () -> MathUtil.applyDeadband(leftJoystick.getY(), Constants.Chassis.JOYSTICK_DEADBAND),
      () -> MathUtil.applyDeadband(rightJoystick.getY(), Constants.Chassis.JOYSTICK_DEADBAND)), m_chassis));
    
    m_chassis.setDefaultCommand(new RunCommand(() -> m_chassis.drive(
      () -> MathUtil.applyDeadband(leftJoystick.getY(), Constants.Chassis.JOYSTICK_DEADBAND),
      () -> MathUtil.applyDeadband(rightJoystick.getY(), Constants.Chassis.JOYSTICK_DEADBAND)), m_chassis));

    // WITH COMMAND FILES
    // m_xbox.leftTrigger().whileTrue(new RunIntake(m_shooter));
    // m_xbox.rightTrigger().whileTrue(new RunEject(m_shooter));

    // leftJoystick.button(1).or(rightJoystick.button(1)).whileTrue(new DriveStraight(()->leftJoystick.getY(), ()->rightJoystick.getY(), m_chassis));
    // leftJoystick.button(2).or(rightJoystick.button(2)).whileTrue(new DriveSlow(()->leftJoystick.getY(), ()->rightJoystick.getY(), m_chassis));
    // m_chassis.setDefaultCommand(new Drive(()->leftJoystick.getY(), ()->rightJoystick.getY(), m_chassis));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
