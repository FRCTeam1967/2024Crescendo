// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj2.command.*; //NOT WORKING
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.*;
import frc.robot.Constants.*;

public class RobotContainer {
  private final Climb leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID), rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID);
  private final Pivot pivot = new Pivot();
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Vision vision = new Vision();
  private final Shooter shooter = new Shooter();
  
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  
  public ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight"), matchTab = Shuffleboard.getTab("Match");
  
  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);
  
  /** Creates RobotContainer, which contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    leftClimb.configDashboard(matchTab);
    rightClimb.configDashboard(matchTab);
    
    matchTab.addDouble("Left Climb Current",
      () -> pdh.getCurrent(Constants.Climb.LEFT_MOTOR_PDH_PORT)).withWidget(BuiltInWidgets.kGraph);
    matchTab.addDouble("Right Climb Current",
      () -> pdh.getCurrent(Constants.Climb.RIGHT_MOTOR_PDH_PORT)).withWidget(BuiltInWidgets.kGraph);
    vision.configDashboard(limelightTab);
    
    resetSwerveSensors();

    CanandEventLoop.getInstance();
    maintainPivotPosition();
    pivot.setBrakeMode();
    
    configureBindings();
  }

  public void setClimbEncoderOffset(){
    leftClimb.setEncoderOffset();
    rightClimb.setEncoderOffset();
  }
  
  public void maintainPivotPosition(){
    pivot.setRelToAbs();
    
    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getAbsPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getAbsPos();
  }
  
  public void resetSwerveSensors() {
    swerve.frontLeft.resetEncoder();
    swerve.frontRight.resetEncoder();
    swerve.backLeft.resetEncoder();
    swerve.backRight.resetEncoder();
    swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
      swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    });;
  }

  public InstantCommand swerveToCoastMode(){
    return new InstantCommand(() -> swerve.setNeutralMode(false), swerve);         
  }

  public void swerveToNeutralMode(){
    swerve.setNeutralMode(true);
  }
  
  /** Defines trigger -> command mappings for {@link CommandXboxController Xbox} controllers */
  private void configureBindings() {
    /*SWERVE*/
    driverController.b().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve));
    driverController.a().onTrue(new VisionAlign(swerve, vision));
    //driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getLeftY(), () -> -driverController.getLeftX(), ()-> 0));
    driverController.leftTrigger().whileTrue(new RunCommand(() -> swerve.goToAngle(180), swerve));
    driverController.rightTrigger().whileTrue(new RunCommand(() -> swerve.goToAngle(90), swerve));
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getLeftY(),
      () -> -driverController.getLeftX(), () -> -driverController.getRightX()));
    
    /*COMBINED INTAKE + PIVOT*/
    operatorController.a().whileTrue(new SequentialCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new RunIntake(intake, -0.5)));
    operatorController.a().whileFalse(new SequentialCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE), new RunIntake(intake, 0)));
    intake.setDefaultCommand(new RunIntake(intake, 0));
    
    /*COMBINED FEEDER + SHOOTER*/
    //operatorController.x().whileTrue(new SequentialCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED, Constants.Feeder.FEED_SPEED).withTimeout(Constants.Feeder.FEED_TIME), new RunShooter(shooter, -(Constants.Shooter.TOP_LEFT_SPEED), Constants.Shooter.TOP_RIGHT_SPEED, -(Constants.Shooter.BOTTOM_LEFT_SPEED), Constants.Shooter.BOTTOM_RIGHT_SPEED)));
    shooter.setDefaultCommand(new RunShooter(shooter, 0, 0, 0, 0));
    
    /*SHOOTER*/
    operatorController.leftTrigger().whileTrue(new RunShooter(shooter, (Constants.Shooter.AMP_TOP_VELOCITY), (Constants.Shooter.AMP_TOP_ACCELERATION), (Constants.Shooter.AMP_BOTTOM_VELOCITY), Constants.Shooter.AMP_BOTTOM_ACCELERATION));
    operatorController.rightTrigger().whileTrue(new RunShooter(shooter, (Constants.Shooter.SPEAKER_TOP_VELOCITY), (Constants.Shooter.SPEAKER_TOP_ACCELERATION), (Constants.Shooter.SPEAKER_BOTTOM_VELOCITY), Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION));
    
    /*FEEDER*/
    //feeder.setDefaultCommand(new RunFeeder(feeder, 0, 0));
    //operatorController.x().whileTrue(new RunFeeder(feeder, -0.8, -0.8));
    
    /*CLIMB*/
    operatorController.x().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.switchMode(), leftClimb), 
      new InstantCommand(() -> rightClimb.switchMode(), rightClimb)));
    operatorController.y().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.TOP_ROTATIONS, false), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.TOP_ROTATIONS, false), rightClimb)));
    operatorController.b().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.SAFE_ROTATIONS, false), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.SAFE_ROTATIONS, false), rightClimb)));
    operatorController.a().onTrue(new ParallelCommandGroup (
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.LATCH_ROTATIONS, true), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.LATCH_ROTATIONS, true), rightClimb)));
    // could replace Climb to latch
    // operatorController.a().onTrue(new ParallelCommandGroup(
    //   new LowerClimbUntilSpike(() -> pdh.getCurrent(Constants.Climb.LEFT_MOTOR_PDH_PORT), leftClimb).withTimeout(Constants.Climb.LOWER_TIME), 
    //   new LowerClimbUntilSpike(() -> pdh.getCurrent(Constants.Climb.RIGHT_MOTOR_PDH_PORT), rightClimb).withTimeout(Constants.Climb.LOWER_TIME)));
    leftClimb.setDefaultCommand(new RunCommand(() -> leftClimb.moveAt(
      () -> MathUtil.applyDeadband(operatorController.getLeftY(), Constants.Climb.DEADBAND)), leftClimb));
    rightClimb.setDefaultCommand(new RunCommand(() -> rightClimb.moveAt(
      () -> MathUtil.applyDeadband(operatorController.getRightY(), Constants.Climb.DEADBAND)), rightClimb));
  }

  /**
   * Passes the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("sShapeAuto");
  }
}
