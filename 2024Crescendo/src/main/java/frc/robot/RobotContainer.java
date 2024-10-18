// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
import frc.robot.subsystems.Vision;
import frc.robot.LimelightHelpers;

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

  public RobotContainer() { 
    resetSensors();

    CanandEventLoop.getInstance();

    configureBindings();

    vision.configDashboard(matchTab);
    swerve.configDashboard(matchTab);

    autoChooserLOL = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser lol", autoChooserLOL);
    
    // autoPathChooser2.setDefaultOption(redFourNote, redFourNote);
    // autoPathChooser2.addOption(BlueCenterDisrupt, BlueCenterDisrupt);
    // autoPathChooser2.addOption(RedCenterDisrupt, RedCenterDisrupt);
    // autoPathChooser2.addOption(redThreeNote, redThreeNote);
    // autoPathChooser2.addOption(blueThreeNote, blueThreeNote);
    // autoPathChooser2.addOption(blueFourNote, blueFourNote);
    // autoPathChooser2.addOption(twoNote, twoNote);
    // autoPathChooser2.addOption(leave, leave);
    // autoPathChooser2.addOption(frontShootSit, frontShootSit);
    // autoPathChooser2.addOption(leftSideSit, leftSideSit);
    // autoPathChooser2.addOption(rightSideSit, rightSideSit);
    // autoPathChooser2.addOption(leftSideLeave, leftSideSit);
    // autoPathChooser2.addOption(rightSideLeave, rightSideSit);
    // autoPathChooser2.addOption(doNothing, doNothing);
    // matchTab.add("Auto Path Chooser 2", autoPathChooser2).withWidget(BuiltInWidgets.kComboBoxChooser);
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
    driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    //driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve)); 
    //driverController.a().onTrue(new AmpReverse(swerve, redAlliance));
    
    driverController.b().onTrue(new VisionAlign(swerve, vision));

    driverController.y().onTrue(new VisionAlignZ(swerve, vision));

    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    //adjust for blue alliance
    driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));

    driverController.povUp().whileTrue(new SwerveDrive(swerve, () -> 0.2, () -> 0, () -> 0));

    driverController.povDown().whileTrue(new SwerveDrive(swerve, () -> -0.2, () -> 0, () -> 0));

    driverController.povRight().whileTrue(new SwerveDrive(swerve, () -> 0, () -> 0.2, () -> 0));

    driverController.povLeft().whileTrue(new SwerveDrive(swerve, () -> 0, () -> -0.2, () -> 0));
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

  //megatag localization from limelight docs  //TODO: figure this out
  LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
  {
    doRejectUpdate = true;
  }
  if(mt2.tagCount == 0)
  {
    doRejectUpdate = true;
  }
  if(!doRejectUpdate)
  {
    swerve.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    swerve.m_poseEstimator.addVisionMeasurement(
        mt2.pose,
        mt2.timestampSeconds);
  }
}
