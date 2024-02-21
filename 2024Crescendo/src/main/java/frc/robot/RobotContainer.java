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

import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.*;
import frc.robot.Constants.*;

public class RobotContainer {
  private final Pivot pivot = new Pivot();
  public final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();
  //public Vision vision = new Vision();
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);

  private final Command turnToAngle = new RunCommand(() -> {
    swerve.goToAngle(100);
  }, swerve);

  
  private final Command goToDefenseMode = new InstantCommand(() -> {
    swerve.defenseMode();
  }, swerve);

  private final Command resetGyro = new InstantCommand(() -> {
    swerve.resetGyro();
  }, swerve);


  //private final CommandXboxController m_xbox = new CommandXboxController(1);
 


  
    

  public RobotContainer() {
  
    resetSensors();
    // pivot.pivotHoming();
    // CanandEventLoop.getInstance();

    CanandEventLoop.getInstance();
    maintainPivotPosition();
    pivot.setBrakeMode();

    configureBindings();
    // maintainPosition();
    // vision.configDashboard(limelightTab);

    
  }

  
  public void maintainPivotPosition(){
    pivot.setRelToAbs();

    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getAbsPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getAbsPos();
  }

  private void configureBindings() {

    //m_xbox.leftTrigger().whileTrue(new RunCommand (() -> intake.runMotors(Constants.Intake.EJECT_ROLLER_SPEED), intake));
    //m_xbox.rightTrigger().whileTrue(new RunCommand (() -> intake.runMotors(Constants.Intake.INTAKE_ROLLER_SPEED), intake));

    //intake.setDefaultCommand(new RunCommand (() -> intake.runMotors(0), intake));

    //left trigger button
    //driverController.leftTrigger().whileTrue(turnToAngle);

    //b button
    driverController.button(2).onTrue(resetGyro);

    //x button
    driverController.button(3).onTrue(goToDefenseMode);

    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()-> 0));

     //a button
     //driverController.a().onTrue(new VisionAlign(swerve, vision));

    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4)));
  /*OPERATOR CONTROLLER */
  operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new ParallelCommandGroup(new RunIntake(intake, Constants.Intake.INTAKE_ROLLER_SPEED), new RunFeeder(feeder, -(Constants.Feeder.FEED_SPEED), 0.0)));
  operatorController.y().whileTrue(new ParallelCommandGroup(new RunFeeder(feeder, 0.0, -(Constants.Feeder.FEED_SPEED)), new RunShooter(shooter, Constants.Shooter.SPEAKER_TOP_VELOCITY, Constants.Shooter.SPEAKER_TOP_ACCELERATION, Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION)));
  operatorController.a().whileTrue(new ParallelCommandGroup(new RunFeeder(feeder, 0.0, -(Constants.Feeder.FEED_SPEED)), new RunShooter(shooter, Constants.Shooter.AMP_TOP_VELOCITY, Constants.Shooter.AMP_TOP_ACCELERATION, Constants.Shooter.AMP_BOTTOM_VELOCITY, Constants.Shooter.AMP_BOTTOM_ACCELERATION)));


    
    //krakenShooter.setDefaultCommand(new RunKrakenShooter(krakenShooter, 0, 0, 0, 0));
    //m_xbox.rightTrigger().whileTrue(new RunKrakenShooter(krakenShooter, -(Constants.KrakenShooter.TOP_LEFT_SPEED), (Constants.KrakenShooter.TOP_RIGHT_SPEED), -(Constants.KrakenShooter.BOTTOM_LEFT_SPEED), Constants.KrakenShooter.BOTTOM_RIGHT_SPEED));
    
  
    //m_xbox.start().onTrue(new HomePivot(pivot));
    //m_xbox.y().whileTrue(new MovePivot(pivot, Constants.Pivot.DEGREE_110)); 
    //m_xbox.b().whileTrue(new MovePivot(pivot, Constants.Pivot.TEST_20)); 
    //m_xbox.y().whileTrue(new MovePivot(pivot, Constants.Pivot.TEST_70)); 

    
    //m_xbox.a().whileTrue(lowerAndIntake);

    //combined intake pivot
    //m_xbox.a().whileTrue(new SequentialCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new RunCommand (() -> intake.runMotors(Constants.Intake.EJECT_ROLLER_SPEED), intake)));
    //m_xbox.a().whileFalse(new SequentialCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE), new RunCommand (() -> intake.runMotors(0), intake)));
  
  }

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
