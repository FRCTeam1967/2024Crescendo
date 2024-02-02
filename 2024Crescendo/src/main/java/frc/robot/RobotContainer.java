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

    configureBindings();

    maintainPosition();
    leftClimb.home();
    rightClimb.home();
    
    matchTab.addDouble("Left PDH Current",
      () -> pdh.getCurrent(Constants.Climb.LEFT_MOTOR_PDH_PORT)).withWidget(BuiltInWidgets.kGraph);
    matchTab.addDouble("Right PDH Current",
      () -> pdh.getCurrent(Constants.Climb.RIGHT_MOTOR_PDH_PORT)).withWidget(BuiltInWidgets.kGraph);
  }

  //TODO: combine with pivot's maintainPosition
  public void maintainPosition(){
    leftClimb.setpoint.velocity = 0;
    leftClimb.setpoint.position = leftClimb.getRelPos();
    leftClimb.goal.velocity = 0;
    leftClimb.goal.position = leftClimb.getRelPos();

    rightClimb.setpoint.velocity = 0;
    rightClimb.setpoint.position = rightClimb.getRelPos();
    rightClimb.goal.velocity = 0;
    rightClimb.goal.position = rightClimb.getRelPos();
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
      new InstantCommand(() -> leftClimb.switchMode(), leftClimb), 
      new InstantCommand(() -> rightClimb.switchMode(), rightClimb)));

    xbox.y().onTrue(new ParallelCommandGroup(
      new ClimbToHeight(Constants.Climb.MAX_WINCH_ROTATIONS, leftClimb),
      new ClimbToHeight(Constants.Climb.MAX_WINCH_ROTATIONS, rightClimb)));
    
    xbox.b().onTrue(new ParallelCommandGroup(
      new ClimbToHeight(Constants.Climb.LATCH_POSITION_ROTATIONS, leftClimb),
      new ClimbToHeight(Constants.Climb.LATCH_POSITION_ROTATIONS, rightClimb)));
    
    xbox.a().onTrue(new ParallelCommandGroup(
      new ClimbToHeight(Constants.Climb.LOW_WINCH_ROTATIONS, leftClimb),
      new ClimbToHeight(Constants.Climb.LOW_WINCH_ROTATIONS, leftClimb)));;
    
    /* could replace ClimbToLowHeight method */
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
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
