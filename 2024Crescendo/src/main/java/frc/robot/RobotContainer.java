// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

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
  
  private final Climb leftClimb, rightClimb;
  
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  private final CommandXboxController xbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public ShuffleboardTab matchTab;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID);
    rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID);

    matchTab = Shuffleboard.getTab("Match");
    leftClimb.configDashboard(matchTab);

    configureBindings();

    leftClimb.homeAtTop();
    rightClimb.homeAtTop();
    
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
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    xbox.x().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.switchMode(), leftClimb), 
      new InstantCommand(() -> rightClimb.switchMode(), rightClimb)));

    xbox.y().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.TOP_HEIGHT, false), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.TOP_HEIGHT, false), rightClimb)));
    
    xbox.a().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.LOW_HEIGHT, false), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.LOW_HEIGHT, false), rightClimb)));

    xbox.b().onTrue(new ParallelCommandGroup (
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.LATCH_HEIGHT, true), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.LATCH_HEIGHT, true), rightClimb)));
    
    /* could replace ClimbToLowHeight method */
    // xbox.a().onTrue(new ParallelCommandGroup(
    //   new LowerClimbUntilSpike(() -> pdh.getCurrent(Constants.Climb.LEFT_MOTOR_PDH_PORT), leftClimb).withTimeout(Constants.Climb.LOWER_TIME), 
    //   new LowerClimbUntilSpike(() -> pdh.getCurrent(Constants.Climb.RIGHT_MOTOR_PDH_PORT), rightClimb).withTimeout(Constants.Climb.LOWER_TIME)));
    
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
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
