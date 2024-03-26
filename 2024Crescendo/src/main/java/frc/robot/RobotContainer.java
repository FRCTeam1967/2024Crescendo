// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Optional;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.commands.*;
import frc.robot.Constants.*;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final Climb leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID);
  private final Climb rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID);
  private final Pivot pivot = new Pivot();
  public final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();
  private Vision vision = new Vision();

  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  SendableChooser<String> autoPathChooser2 = new SendableChooser<String>(); 
  public static final String redFourNote = "RedFourNote";
  public static final String blueFourNote = "BlueFourNote";
  public static final String twoNote = "TwoNote";
  public static final String leave = "Leave";
  public static final String frontShootSit = "FrontShootSit";
  public static final String leftShootSit = "LeftShootSit";
  public static final String rightShootSit = "RightShootSit";
  public static final String leftShootLeave = "LeftShootLeave";
  public static final String rightShootLeave = "RightShootLeave";
  
  public static boolean redAlliance;
  String autoPath;

  public RobotContainer() {
    resetSensors();

    CanandEventLoop.getInstance();
    maintainPivotPosition();
    pivot.setBrakeMode();

    configureBindings();

    vision.configDashboard(matchTab);
    shooter.configDashboard(matchTab);
    feeder.configDashboard(matchTab);
    swerve.configDashboard(matchTab);
    leftClimb.configDashboard(matchTab);
    rightClimb.configDashboard(matchTab);

    autoPathChooser2.setDefaultOption(redFourNote, redFourNote);
    autoPathChooser2.addOption(blueFourNote, blueFourNote);
    autoPathChooser2.addOption(twoNote, twoNote);
    autoPathChooser2.addOption(leave, leave);
    autoPathChooser2.addOption(frontShootSit, frontShootSit);
    autoPathChooser2.addOption(leftShootSit, leftShootSit);
    autoPathChooser2.addOption(rightShootSit, rightShootSit);
    autoPathChooser2.addOption(leftShootLeave, leftShootLeave);
    autoPathChooser2.addOption(rightShootLeave, rightShootLeave);
    matchTab.add("Auto Path Chooser 2", autoPathChooser2).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void onEnable(Optional<Alliance> alliance){
    if (alliance.get() == Alliance.Red) redAlliance = true;
    else redAlliance = false;
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
  
  private void configureBindings() {
    //DEFAULT COMMANDS
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4)));
    
    leftClimb.setDefaultCommand(new ManualClimb(() -> operatorController.getRightY(), leftClimb)); //flipped for field orientation
    rightClimb.setDefaultCommand(new ManualClimb(() -> operatorController.getLeftY(), rightClimb));
    
    intake.setDefaultCommand(new RunIntake(intake, 0));
    feeder.setDefaultCommand(new RunFeeder(feeder, 0));
    
    //CHASSIS
    driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    //driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve)); 
    driverController.a().onTrue(new AmpReverse(swerve, redAlliance));
    
    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));
    
    //INTAKE + PIVOT + FEEDER
    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunPivotIntakeBeam(pivot, intake, feeder),
      new ReverseBeamFeeder(feeder),
      new RumbleController(driverController, operatorController).withTimeout(2)
    ));
    
    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileFalse(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE));
    operatorController.rightBumper().whileTrue(new ParallelCommandGroup(new RunIntake(intake, -Constants.Intake.INTAKE_ROLLER_SPEED), new RunFeeder(feeder, -Constants.Feeder.FEED_SPEED)));
    
    //SHOOTER + FEEDER
    operatorController.y().whileTrue(new ShootSpeaker(shooter, feeder));
    operatorController.b().whileTrue(new ParallelCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED), new RunShooter(shooter, false)).withTimeout(1.5));
    operatorController.a().onTrue(new SequentialCommandGroup(
      new AmpReverse(swerve,redAlliance),
      new ParallelCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED), new RunShooter(shooter, false)).withTimeout(1)
    ));
    operatorController.start().whileTrue(new RunShooter(shooter, false));
    
    //CLIMB
    operatorController.x().whileTrue(new ParallelCommandGroup(new LowerClimbUntilLatch(leftClimb), new LowerClimbUntilLatch(rightClimb)));
    operatorController.leftBumper().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.changeStatus(), leftClimb), new InstantCommand(() -> rightClimb.changeStatus(), rightClimb)
    ));
  }

  public Command getAutonomousCommand() {
    autoPath = autoPathChooser2.getSelected();
    
    switch (autoPath) {
      case redFourNote:
        return RedFourNote();
      case blueFourNote:
        return BlueFourNote();
      case twoNote:
        return TwoNote();
      case leave:
        return Leave();
      case frontShootSit:
        return FrontShootSit();
      case leftShootSit:
        return LeftShootSit();
      case rightShootSit:
        return RightShootSit();
      case leftShootLeave:
        return LeftShootLeave();
      case rightShootLeave:
        return RightShootLeave();
      default:
        return FrontShootSit();
    }
  }

  /* USED WHILE FIGURING OUT TRAJECTORY/PATH PLANNER */
  /*public Command RightAngleShoot(){
    return new SequentialCommandGroup(
      new SwerveDrive(swerve, ()->0, ()->0, ()->-0.2).withTimeout(0.25),
      new ShootSpeaker(shooter, feeder)
    ).withTimeout(5);
  }

  public Command LeftAngleShoot(){
      return new SequentialCommandGroup(
        new SwerveDrive(swerve, ()->0, ()->0, ()->0.2).withTimeout(0.25),
        new ShootSpeaker(shooter, feeder)
      ).withTimeout(5);
  }*/

  public Command FrontShootSit(){
    return new ShootSpeaker(shooter, feeder).withTimeout(3);
  }

  public Command LeftShootSit(){
    swerve.setGyroAngle(Constants.Auto.LEFT_ANGLE);
    return new ShootSpeaker(shooter, feeder).withTimeout(3);
  }

  public Command RightShootSit(){
    swerve.setGyroAngle(Constants.Auto.RIGHT_ANGLE);
    return new ShootSpeaker(shooter, feeder).withTimeout(3);
  }

  public Command LeftShootLeave(){
    swerve.setGyroAngle(Constants.Auto.LEFT_ANGLE);
    return new SequentialCommandGroup(new ShootSpeaker(shooter, feeder).withTimeout(3), new SwerveDrive(swerve, ()->0.5, ()->0, ()->0).withTimeout(1.5));
  }

  public Command RightShootLeave(){
    swerve.setGyroAngle(Constants.Auto.RIGHT_ANGLE);
    return new SequentialCommandGroup(new ShootSpeaker(shooter, feeder).withTimeout(3), new SwerveDrive(swerve, ()->0.5, ()->0, ()->0).withTimeout(1.5));
  }
  
  public Command Leave(){
    return new SwerveDrive(swerve, ()->0.5, ()->0, ()->0).withTimeout(1.5);
  }

  public Command TwoNote(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
      new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5));
  }

  public Command BlueFourNote(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
      new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.59, ()->0.68, ()->0)).withTimeout(1.3),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.59, ()->-0.68, ()->0)).withTimeout(1.32),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.569, ()->-0.656, ()->0)).withTimeout(1.35),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.569, ()->0.656, ()->0)).withTimeout(1.4),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5));
  }

  public Command RedFourNote(){
    return new SequentialCommandGroup(
      new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
      new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.59, ()->-0.68, ()->0)).withTimeout(1.3),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.59, ()->0.68, ()->0)).withTimeout(1.32),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.569, ()->0.656, ()->0)).withTimeout(1.35),
      new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.569, ()->-0.656, ()->0)).withTimeout(1.4),
      new ShootSpeaker(shooter, feeder).withTimeout(1.5));
  }

  public void resetSensors() {
    swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));

    swerve.frontLeft.resetEncoder();
    swerve.frontRight.resetEncoder();
    swerve.backLeft.resetEncoder();
    swerve.backRight.resetEncoder();
    swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
      swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    });;
  }

  public void resetSwerveGyro(){
    swerve.resetGyro();
  }
}