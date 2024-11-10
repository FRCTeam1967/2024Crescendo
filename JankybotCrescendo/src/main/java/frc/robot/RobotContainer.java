// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.AmpBar;

public class RobotContainer {
  public final Swerve swerve = new Swerve();
  public final Shooter shooter = new Shooter();
  public final AmpBar ampBar = new AmpBar();
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
    matchTab.add("auto chooser lol", autoChooserLOL);
    
    pdh.setSwitchableChannel(true);
  }

  public void onEnable(Optional<Alliance> alliance){
    if (alliance.get() == Alliance.Red) redAlliance = true;
    else redAlliance = false;
  }
  public void maintainAmpBarPosition(){
    ampBar.setPosition(0);

    ampBar.setpoint.velocity = 0;
    ampBar.setpoint.position = 0;
    ampBar.goal.velocity = 0;
    ampBar.goal.position = 0;
  }

  private void configureBindings() {
    //DEFAULT COMMANDS
   swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
     () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4)));

    //CHASSIS
     driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve));
    
    driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    //adjust for blue alliance
    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->120));
    
    // driverController.a().onTrue(new AmpReverse(swerve, redAlliance));
    
    // driverController.b().onTrue(new VisionAlign(swerve, vision));

    // driverController.y().onTrue(new VisionAlignZ(swerve, vision));


    driverController.povDown().whileTrue(new SwerveDrive(swerve, () -> 0.4, () -> 0, () -> 0));

    // driverController.povDown().whileTrue(new SwerveDrive(swerve, () -> -0.4, () -> 0, () -> 0));

    // driverController.povRight().whileTrue(new SwerveDrive(swerve, () -> 0, () -> 0.2, () -> 0));

    // driverController.povLeft().whileTrue(new SwerveDrive(swerve, () -> 0, () -> -0.2, () -> 0));
  // //SHOOTER
      operatorController.y().whileTrue(new RunShooter(shooter, true));
      operatorController.rightTrigger().whileTrue(new RunIntake(-Constants.Shooter.SPEAKER_TOP_ACCELERATION, shooter));
      operatorController.x().whileTrue(new ShootAcrossField(shooter));
  
  // AMP
    operatorController.a().whileTrue(new SequentialCommandGroup((new RunShooter(shooter, false)).withTimeout(0.5), 
    //new WaitCommand(0.2), 
    new MoveAmpBar(ampBar, Constants.AmpBar.AMP_UP))); // waitcommand was 0.2
    operatorController.a().whileFalse(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_SAFE));

    operatorController.a().whileTrue(new ParallelCommandGroup(new SequentialCommandGroup(new WaitCommand(0.05).withTimeout(0.05), new RunShooter(shooter, false)).withTimeout(1.0), 
    //new WaitCommand(0.2), 
    new MoveAmpBar(ampBar, Constants.AmpBar.AMP_UP))); // waitcommand was 0.2
    operatorController.a().whileFalse(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_SAFE));
    
    //operatorController.b().whileTrue(new RunShooter(shooter,  false));
  }
  public void resetSensors() {
    // swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));
    swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));


    swerve.frontLeft.resetEncoder();
    swerve.frontRight.resetEncoder();
    swerve.backLeft.resetEncoder();
    swerve.backRight.resetEncoder();
    // swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
    //   swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    // });;

    swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
      swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    });;
  }
  
  public Command Leave() {
    return new SwerveDrive(swerve, () -> 0.5, () -> 0, () -> 0).withTimeout(2.5);
  }

  public Command getAutonomousCommand() {
    return autoChooserLOL.getSelected();
  }
}