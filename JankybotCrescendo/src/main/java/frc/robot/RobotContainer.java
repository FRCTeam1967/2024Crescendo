// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  public final Swerve swerve = new Swerve();
  public final Shooter shooter = new Shooter();
  private SendableChooser<Command> autoChooserLOL;

  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);

  public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  public static boolean redAlliance;

  public RobotContainer() {
  NamedCommands.registerCommand("intake", new RunIntake(-Constants.Shooter.SPEAKER_TOP_ACCELERATION, shooter).withTimeout(1.0));
  NamedCommands.registerCommand("shootSpeaker", new RunShooter(shooter, true).withTimeout(5.0));
    resetSensors();
    CanandEventLoop.getInstance();

    configureBindings();
    swerve.configDashboard(matchTab);

    autoChooserLOL = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser lol", autoChooserLOL);

    pdh.setSwitchableChannel(true);
  }

  public void onEnable(Optional<Alliance> alliance){
    if (alliance.get() == Alliance.Red) redAlliance = true;
    else redAlliance = false;
  }

  private void configureBindings() {
    //DEFAULT COMMANDS
   swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
     () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4)));

    //CHASSIS
     driverController.start().onTrue(new InstantCommand(() -> swerve.resetpGyro(), swerve));
    driverController.leftTrigger().onTrue(new InstantCommand(() -> swerve.resetpGyro(), swerve));
    driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve)); 
    
    // driverController.a().onTrue(new AmpReverse(swerve, redAlliance));
    
    // driverController.b().onTrue(new VisionAlign(swerve, vision));

    // driverController.y().onTrue(new VisionAlignZ(swerve, vision));

    //driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    //adjust for blue alliance
    // driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));

    // driverController.povUp().whileTrue(new SwerveDrive(swerve, () -> 0.2, () -> 0, () -> 0));

    // driverController.povDown().whileTrue(new SwerveDrive(swerve, () -> -0.2, () -> 0, () -> 0));

    // driverController.povRight().whileTrue(new SwerveDrive(swerve, () -> 0, () -> 0.2, () -> 0));

    // driverController.povLeft().whileTrue(new SwerveDrive(swerve, () -> 0, () -> -0.2, () -> 0));
  // //SHOOTER
      operatorController.y().whileTrue(new RunShooter(shooter, true));
      operatorController.a().whileTrue(new RunShooter(shooter, false));
      operatorController.rightTrigger().whileTrue(new RunIntake(-Constants.Shooter.SPEAKER_TOP_ACCELERATION, shooter));
      operatorController.x().whileTrue(new ShootAcrossField(shooter));
  
  }

  public void resetSensors() {
    // swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));
    swerve.resetpOdometry(new Pose2d(0.0, 0.0, swerve.pGetRotation2d()));


    swerve.frontLeft.resetEncoder();
    swerve.frontRight.resetEncoder();
    swerve.backLeft.resetEncoder();
    swerve.backRight.resetEncoder();
    // swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
    //   swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    // });;

    swerve.pOdometry.update(swerve.pGetRotation2d(), new SwerveModulePosition[] {
      swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    });;
  }

  public Command getAutonomousCommand() {
    return autoChooserLOL.getSelected();
  }
}