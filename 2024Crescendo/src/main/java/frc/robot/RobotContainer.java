// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.pathplanner.lib.commands.PathPlannerAuto;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Pivot pivot = new Pivot();
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Vision vision = new Vision();
  private final KrakenShooter shooter = new KrakenShooter();
  
  public ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight tab");
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final Command turnToAngle = new RunCommand(() -> swerve.goToAngle(100), swerve);
  private final Command goToDefenseMode = new InstantCommand(() -> swerve.defenseMode(), swerve);
  private final Command resetGyro = new InstantCommand(() -> swerve.resetGyro(), swerve);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    resetSensors();
    
    CanandEventLoop.getInstance();

    pivot.pivotHoming();
    maintainPosition();
    
    vision.configDashboard(limelightTab);
    
    configureBindings();
    pivot.setBrakeMode();
  }

  public void maintainPivotPosition(){
    pivot.pivotHoming();
    
    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getAbsPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getAbsPos();
  }

  private void configureBindings() {
    //shooter
    operatorController.leftTrigger().whileTrue(new RunKrakenShooter(shooter, (Constants.KrakenShooter.AMP_TOP_VELOCITY), (Constants.KrakenShooter.AMP_TOP_ACCELERATION), (Constants.KrakenShooter.AMP_BOTTOM_VELOCITY), Constants.KrakenShooter.AMP_BOTTOM_ACCELERATION));
    operatorController.rightTrigger().whileTrue(new RunKrakenShooter(shooter, (Constants.KrakenShooter.SPEAKER_TOP_VELOCITY), (Constants.KrakenShooter.SPEAKER_TOP_ACCELERATION), (Constants.KrakenShooter.SPEAKER_BOTTOM_VELOCITY), Constants.KrakenShooter.SPEAKER_BOTTOM_ACCELERATION));
    
    //feeder
    //feeder.setDefaultCommand(new RunFeeder(feeder, 0, 0));
    //m_xbox.x().whileTrue(new RunFeeder(feeder, -0.8, -0.8));
    //left trigger button
    //m_driverController.leftTrigger().whileTrue(turnToAngle);

    driverController.button(2).onTrue(resetGyro); //b button
    driverController.button(3).onTrue(goToDefenseMode); //x button
    driverController.a().onTrue(new VisionAlign(swerve, vision)); //a button
    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -m_driverController.getRawAxis(1), () -> -m_driverController.getRawAxis(0), ()-> 0));

    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -m_driverController.getRawAxis(1),
      () -> -m_driverController.getRawAxis(0), () -> -m_driverController.getRawAxis(4)));

    //combined intake pivot
    operatorController.a().whileTrue(new SequentialCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new RunIntake(intake, -0.5)));
    operatorController.a().whileFalse(new SequentialCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE), new RunIntake(intake, 0)));
    intake.setDefaultCommand(new RunIntake(intake, 0));
    
    //combined feeder shooter
    //m_xbox.x().whileTrue(new SequentialCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED, Constants.Feeder.FEED_SPEED).withTimeout(Constants.Feeder.FEED_TIME), new RunKrakenShooter(shooter, -(Constants.KrakenShooter.TOP_LEFT_SPEED), Constants.KrakenShooter.TOP_RIGHT_SPEED, -(Constants.KrakenShooter.BOTTOM_LEFT_SPEED), Constants.KrakenShooter.BOTTOM_RIGHT_SPEED)));
    shooter.setDefaultCommand(new RunKrakenShooter(shooter, 0, 0, 0, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return new PathPlannerAuto("sShapeAuto");
   
  }

  public void resetSensors() {
      swerve.frontLeft.resetEncoder();
      swerve.frontRight.resetEncoder();
      swerve.backLeft.resetEncoder();
      swerve.backRight.resetEncoder();
      swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
            swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
          });;
  }
}
