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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
  private final Climb leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID, Constants.Climb.LEFT_ENCODER_ID);
  private final Climb rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID, Constants.Climb.RIGHT_ENCODER_ID);

  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final CommandXboxController xbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public ShuffleboardTab matchTab;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    matchTab = Shuffleboard.getTab("Match");
    leftClimb.configDashboard(matchTab); //when testing, do one side at a time?
    leftClimb.home();
    rightClimb.home();

    configureBindings();
    
    matchTab.addDouble("Left PDH Current",
      () -> pdh.getCurrent(Constants.Climb.LEFT_MOTOR_PDH_PORT)).withWidget(BuiltInWidgets.kGraph);
    matchTab.addDouble("Right PDH Current",
      () -> pdh.getCurrent(Constants.Climb.RIGHT_MOTOR_PDH_PORT)).withWidget(BuiltInWidgets.kGraph);
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
    xbox.x().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.safeToClimb(), leftClimb), 
      new InstantCommand(() -> rightClimb.safeToClimb(), rightClimb)));

    xbox.y().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.MAX_WINCH_ROTATIONS), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.MAX_WINCH_ROTATIONS), rightClimb)));
    
    xbox.b().onTrue(new ParallelCommandGroup (
      new InstantCommand(() -> leftClimb.moveAt(() -> Constants.Climb.LATCH_POSITION_ROTATIONS), leftClimb),
      new InstantCommand(() -> rightClimb.moveAt(() -> Constants.Climb.LATCH_POSITION_ROTATIONS), rightClimb)));
    
    /* either of the following 2 triggers will be used, delete the other one */
    xbox.a().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.LOW_WINCH_ROTATIONS), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.LOW_WINCH_ROTATIONS), rightClimb)));
    
    xbox.a().onTrue(new ParallelCommandGroup(
      new LowerClimbUntilSpike(leftClimb, () -> pdh.getCurrent(Constants.Climb.LEFT_MOTOR_PDH_PORT)).withTimeout(Constants.Climb.LOWER_TIME), 
      new LowerClimbUntilSpike(rightClimb, () -> pdh.getCurrent(Constants.Climb.RIGHT_MOTOR_PDH_PORT)).withTimeout(Constants.Climb.LOWER_TIME)));
    
    leftClimb.setDefaultCommand(new RunCommand(() -> leftClimb.moveAt(
      () -> MathUtil.applyDeadband(xbox.getLeftY(), Constants.Climb.DEADBAND)), leftClimb));
    rightClimb.setDefaultCommand(new RunCommand(() -> rightClimb.moveAt(
      () -> MathUtil.applyDeadband(xbox.getRightY(), Constants.Climb.DEADBAND)), rightClimb));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
