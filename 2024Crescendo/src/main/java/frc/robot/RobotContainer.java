// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final Climb m_climb = new Climb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xbox =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public ShuffleboardTab m_matchTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //shuffleboard
    m_matchTab = Shuffleboard.getTab("Match");
    m_climb.configDashboard(m_matchTab);

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
    m_xbox.b().onTrue(new RunCommand(() -> m_climb.changeFactor(Constants.Climb.UNWIND_FACTOR), m_climb));
    m_xbox.a().onTrue(new RunCommand(() -> m_climb.changeFactor(Constants.Climb.WIND_FACTOR), m_climb));
    
    m_climb.setDefaultCommand(new RunCommand(() -> m_climb.moveWinch(
      () -> MathUtil.applyDeadband(m_xbox.getLeftY(), Constants.Climb.DEADBAND),
      () -> MathUtil.applyDeadband(m_xbox.getRightY(), Constants.Climb.DEADBAND)
    ), m_climb));

    m_xbox.x().onTrue(new RunCommand(() -> m_climb.winchToChainHeight(), m_climb));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
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
