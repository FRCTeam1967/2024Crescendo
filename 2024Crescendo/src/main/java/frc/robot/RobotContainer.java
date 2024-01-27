// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
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
  private final TelescopingArm leftArm = new TelescopingArm(Constants.TelescopingArm.LEFT_MOTOR_ID);
  private final TelescopingArm rightArm = new TelescopingArm(Constants.TelescopingArm.RIGHT_MOTOR_ID);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController xbox =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public ShuffleboardTab matchTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //shuffleboard
    matchTab = Shuffleboard.getTab("Match");
    leftArm.configDashboard(matchTab);
    rightArm.configDashboard(matchTab);

    configureBindings();

    maintainPosition();
    leftArm.home();
    rightArm.home();
  }
  
  //TODO: combine with pivot's maintainPosition?
  public void maintainPosition(){
    leftArm.setpoint.velocity = 0;
    leftArm.setpoint.position = leftArm.getRelPos();
    leftArm.goal.velocity = 0;
    leftArm.goal.position = leftArm.getRelPos();

    rightArm.setpoint.velocity = 0;
    rightArm.setpoint.position = rightArm.getRelPos();
    rightArm.goal.velocity = 0;
    rightArm.goal.position = rightArm.getRelPos();
  }
  // public void refreshSensor(){
  //   leftArm.moveTo(leftArm.getRelPos());
  //   rightArm.moveTo(rightArm.getRelPos());
  // }

  // public void stopMotor(){
  //   leftArm.stop();
  //   rightArm.stop();
  // }

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
    xbox.b().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftArm.changeFactor(Constants.TelescopingArm.UNWIND_FACTOR), leftArm), 
      new InstantCommand(() -> rightArm.changeFactor(Constants.TelescopingArm.UNWIND_FACTOR), rightArm)));
    
    xbox.a().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftArm.changeFactor(Constants.TelescopingArm.WIND_FACTOR), leftArm), 
      new InstantCommand(() -> rightArm.changeFactor(Constants.TelescopingArm.WIND_FACTOR), rightArm)));
    
    xbox.x().onTrue(new ParallelCommandGroup(new GoToMaxHeight(leftArm), new GoToMaxHeight(rightArm)));
    
    leftArm.setDefaultCommand(new RunCommand(() -> leftArm.moveWinch(
      () -> MathUtil.applyDeadband(xbox.getLeftY(), Constants.TelescopingArm.DEADBAND)), leftArm));

    rightArm.setDefaultCommand(new RunCommand(() -> rightArm.moveWinch(
      () -> MathUtil.applyDeadband(xbox.getRightY(), Constants.TelescopingArm.DEADBAND)), rightArm));
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
