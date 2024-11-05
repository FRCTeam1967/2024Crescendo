// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Xbox;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.VisionAlignZ;
import frc.robot.commands.WallSnapDrive;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  public final Swerve swerve = new Swerve();
  private final Vision vision = new Vision("new vision");
  private SendableChooser<Command> autoChooserLOL;

  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  SendableChooser<String> autoPathChooser2 = new SendableChooser<String>(); 
  public static final String redThreeNote = "RedThreeNote";
  public static final String BlueCenterDisrupt = "BlueCenterDisrupt";
  public static final String RedCenterDisrupt = "RedCenterDisrupt";
  public static final String blueThreeNote = "BlueThreeNote";
  public static final String redFourNote = "RedFourNote";
  public static final String doNothing = "DoNothing";
  public static final String blueFourNote = "BlueFourNote";
  public static final String twoNote = "TwoNote";
  public static final String leave = "Leave";
  public static final String frontShootSit = "FrontShootSit";
  public static final String leftSideSit = "LeftSideSit";
  public static final String rightSideSit = "RightSideSit";
  public static final String leftSideLeave = "LeftSideLeave";
  public static final String rightSideLeave = "RightSideLeave";

  public static boolean redAlliance;
  String autoPath;
  boolean doRejectUpdate;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public RobotContainer() { 
    resetSensors();

    CanandEventLoop.getInstance();

    configureBindings();

    vision.configDashboard(matchTab);
    swerve.configDashboard(matchTab);

    autoChooserLOL = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser lol", autoChooserLOL);
  }

  public void onEnable(Optional<Alliance> alliance){
    if (alliance.get() == Alliance.Red) redAlliance = true;
    else redAlliance = false;
  }

  private void configureBindings() {
    //DEFAULT COMMANDS
    //swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
      //() -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4)));
    
      //CHASSIS
    driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    // driverController.b().onTrue(new VisionAlign(swerve, vision));
    // driverController.y().onTrue(new VisionAlignZ(swerve, vision));

    //wall-snap-drive
    // driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    // driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));
  }

  public void resetSensors() {
    swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));

    swerve.frontLeft.resetEncoder();
    swerve.frontRight.resetEncoder();
    swerve.backLeft.resetEncoder();
    swerve.backRight.resetEncoder();
    swerve.m_poseEstimator.update(swerve.getRotation2d(), new SwerveModulePosition[] {
      swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    });;
  }

  public void resetSwerveGyro(){
    swerve.resetGyro();
  }

  public Command getAutonomousCommand() {
    return autoChooserLOL.getSelected();
  }

  //**************************************************************//

  //limelight methods for alignment
  //for X alignment (how rotational it should align)
  private double limelight_aim_proportional() {
    double kP = 0.035; //test
    
    //TX -> x-axis offset in degrees, multiply by angular speed to be radians/second
    double targetingAngularVelocity = (LimelightHelpers.getTX("limelight") * kP) * Swerve.kMaxAngularSpeed;
    
    targetingAngularVelocity *= -1.0; //invert because of some positive/negative thing
    return targetingAngularVelocity;
  }

  //for Y alignment (how forward/backward it should go)
  private double limelight_range_proportional() {    
    double kP = 0.1; //test

    //TY -> y-axis offset in degrees, multiply by angular speed to be raidans/second
    double targetingForwardSpeed = (LimelightHelpers.getTY("limelight") * kP) * Swerve.kMaxSpeed;

    targetingForwardSpeed *= -1.0; //invert because of some positive/negative thing //TODO: udnerstand this
    return targetingForwardSpeed;
  }

  //drive for robot container
  public void drive(boolean fieldRelative) {
    var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftY(), 0.02)) * Swerve.kMaxSpeed;
    var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.02)) * Swerve.kMaxSpeed;
    var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX(), 0.02)) * Swerve.kMaxAngularSpeed;

    // while the left-bumper is pressed, overwrite some of the driving values with the output of limelight override method
    if (driverController.leftBumper().getAsBoolean()){
      final var rot_limelight = limelight_aim_proportional();
      rot = rot_limelight;

      final var forward_limelight = limelight_range_proportional();
      xSpeed = forward_limelight;

      //turn off field relative
      fieldRelative = false;
    }

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, 10); //TODO: figure the seconds out?

  }
}
