// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {
  private final Pivot pivot = new Pivot();
  public final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final KrakenShooter krakenShooter = new KrakenShooter();
  public ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight tab");
  public Vision vision = new Vision();


  private final Command turnToAngle = new RunCommand(() -> {
    swerve.goToAngle(100);
  }, swerve);

  
  private final Command goToDefenseMode = new InstantCommand(() -> {
    swerve.defenseMode();
  }, swerve);

  private final Command resetGyro = new InstantCommand(() -> {
    swerve.resetGyro();
  }, swerve);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  //private final CommandXboxController m_xbox = new CommandXboxController(1);
 


  
    

  public RobotContainer() {
  
    resetSensors();
    pivot.pivotHoming();
    CanandEventLoop.getInstance();



    configureBindings();
    maintainPosition();
    vision.configDashboard(limelightTab);

    
  }

  public void maintainPosition(){
    pivot.setpoint.velocity = 0;
    pivot.setpoint.position = pivot.getAbsPos();
    pivot.goal.velocity = 0;
    pivot.goal.position = pivot.getAbsPos();
  }

  public void pivotHoming(){
    pivot.pivotHoming();
  }

  public void setPivotBrakeMode(){
    pivot.setBrakeMode();
  }

  public void refreshSensor(){
    pivot.moveTo(pivot.getRelPos());
  }

  public void stopMotor(){
    pivot.stop();
  }

  private void configureBindings() {

    //m_xbox.leftTrigger().whileTrue(new RunCommand (() -> intake.runMotors(Constants.Intake.EJECT_ROLLER_SPEED), intake));
    //m_xbox.rightTrigger().whileTrue(new RunCommand (() -> intake.runMotors(Constants.Intake.INTAKE_ROLLER_SPEED), intake));

    //intake.setDefaultCommand(new RunCommand (() -> intake.runMotors(0), intake));

    //left trigger button
    //m_driverController.leftTrigger().whileTrue(turnToAngle);

    //b button
    m_driverController.button(2).onTrue(resetGyro);

    //x button
    m_driverController.button(3).onTrue(goToDefenseMode);

    m_driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -m_driverController.getRawAxis(1), () -> -m_driverController.getRawAxis(0), ()-> 0));

     //a button
     m_driverController.a().onTrue(new VisionAlign(swerve, vision));

    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -m_driverController.getRawAxis(1),
      () -> -m_driverController.getRawAxis(0), () -> -m_driverController.getRawAxis(4)));
    

    
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
