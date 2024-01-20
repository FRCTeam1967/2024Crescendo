// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import com.reduxrobotics.canand.CanandEventLoop;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter shooter = new Shooter();
  private final Pivot pivot = new Pivot();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_xbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    pivot.pivotHoming();
    CanandEventLoop.getInstance();
    
    maintainPosition();
  }

  public void maintainPosition(){
    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getRelPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getRelPos();

  }

  public void refreshSensor(){
    pivot.moveTo(pivot.getRelPos());
  }

  public void stopMotor(){
    pivot.stop();
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_xbox.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //m_xbox.rightTrigger().whileTrue(new RunShooter(shooter, 2.0));
    m_xbox.rightTrigger().onTrue(new MovePivot(pivot, Constants.Pivot.DEGREE_90)); 
    m_xbox.leftTrigger().onTrue(new MovePivot(pivot, Constants.Pivot.DEGREE_180));
    m_xbox.a().onTrue(new MovePivot(pivot, Constants.Pivot.DEGREE_0));
    m_xbox.b().onTrue(new MovePivot(pivot, Constants.Pivot.DEGREE_10));
    m_xbox.start().onTrue(new HomePivot(pivot));
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
