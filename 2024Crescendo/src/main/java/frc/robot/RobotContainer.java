// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Xbox;
// import frc.robot.commands.AmpReverse;
import frc.robot.commands.LowerClimbUntilLatch;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.MoveAmpBar;
import frc.robot.commands.MovePivot;
import frc.robot.commands.ReverseBeamFeeder;
import frc.robot.commands.RumbleController;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunPivotIntakeBeam;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.SwerveDrive;
// import frc.robot.commands.VisionAlign;
// import frc.robot.commands.VisionAlignZ;
import frc.robot.commands.WallSnapDrive;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final AmpBar ampBar = new AmpBar();
  private final Climb leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID);
  private final Climb rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID);
  private final Pivot pivot = new Pivot();
  public final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();
  // private final Vision vision = new Vision();
  private SendableChooser<Command> autoChooserLOL;

  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  public static boolean redAlliance;

  public RobotContainer() {
    NamedCommands.registerCommand("pivotSequence", new RunPivotIntakeBeam(pivot, intake, feeder).withTimeout(Constants.Auto.PIVOT_INTAKE_TIMEOUT));
    NamedCommands.registerCommand("pivotDown", new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN).withTimeout(Constants.Auto.PIVOT_INTAKE_TIMEOUT));
    NamedCommands.registerCommand("pivotUp", new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE).withTimeout(Constants.Auto.PIVOT_UP_TIMEOUT));
    NamedCommands.registerCommand("shootSpeaker", new ShootSpeaker(shooter, feeder).withTimeout(1.0));
    
    resetSensors();
    CanandEventLoop.getInstance();
    configureBindings();

    // vision.configDashboard(matchTab);
    // shooter.configDashboard(matchTab);
    // feeder.configDashboard(matchTab);
    swerve.configDashboard(matchTab);
    leftClimb.configDashboard(matchTab);
    rightClimb.configDashboard(matchTab);
    // ampBar.configDashboard(matchTab);

    autoChooserLOL = AutoBuilder.buildAutoChooser();
    matchTab.add("Auto Chooser lol", autoChooserLOL); //TODO: check if this shows on match tab
    SmartDashboard.putData("Auto Chooser lol", autoChooserLOL);
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

  public void maintainAmpBarPosition(){
    ampBar.setPosition(0);

    ampBar.setpoint.velocity = 0;
    ampBar.setpoint.position = 0;
    ampBar.goal.velocity = 0;
    ampBar.goal.position = 0;
  }

  public void movePivotSafe() {
    new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE);
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
  
  private void configureBindings() {
    //DEFAULT COMMANDS
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4)));
    
    leftClimb.setDefaultCommand(new ManualClimb(() -> operatorController.getRightY(), leftClimb));
    rightClimb.setDefaultCommand(new ManualClimb(() -> operatorController.getLeftY(), rightClimb));
    
    intake.setDefaultCommand(new RunIntake(intake, 0));
    feeder.setDefaultCommand(new RunFeeder(feeder, 0));
    //shooter.setDefaultCommand(new InstantCommand(() -> shooter.stopMotors()));
    
    //CHASSIS
    driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));

    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    //adjust for blue alliance
    driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));
    
    // driverController.a().onTrue(new AmpReverse(swerve, redAlliance));

    // driverController.b().onTrue(new VisionAlign(swerve, vision));
    // driverController.y().onTrue(new VisionAlignZ(swerve, vision));

    // driverController.povUp().whileTrue(new SwerveDrive(swerve, () -> 0.2, () -> 0, () -> 0));

    // driverController.povDown().whileTrue(new SwerveDrive(swerve, () -> -0.2, () -> 0, () -> 0));

    // driverController.povRight().whileTrue(new SwerveDrive(swerve, () -> 0, () -> 0.2, () -> 0));

    // driverController.povLeft().whileTrue(new SwerveDrive(swerve, () -> 0, () -> -0.2, () -> 0));

    //testing with one joystick
    // driverController.leftTrigger().or(driverController.rightTrigger()).whileTrue(new SequentialCommandGroup(
    //   new RunPivotIntakeBeam(pivot, intake, feeder),
    //   new ReverseBeamFeeder(feeder),
    //   new RumbleController(driverController, operatorController).withTimeout(2)
    // ));
    // driverController.leftTrigger().or(driverController.rightTrigger()).whileFalse(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE));
    // driverController.rightBumper().whileTrue(new ParallelCommandGroup(new RunIntake(intake, -Constants.Intake.INTAKE_ROLLER_SPEED), new RunFeeder(feeder, -Constants.Feeder.FEED_SPEED)));

    
    //INTAKE
    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunPivotIntakeBeam(pivot, intake, feeder),
      new ReverseBeamFeeder(feeder),
      new RumbleController(driverController, operatorController).withTimeout(2)
    ));
    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileFalse(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE));

    //EJECT
    operatorController.rightBumper().whileTrue(new ParallelCommandGroup(new RunIntake(intake, -Constants.Intake.INTAKE_ROLLER_SPEED), new RunFeeder(feeder, -Constants.Feeder.FEED_SPEED)));
    
    //SHOOTER
    operatorController.y().whileTrue(new ShootSpeaker(shooter, feeder));

    //AMP
    //TODO: test timing
    operatorController.a().whileTrue(new SequentialCommandGroup(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_UP), new WaitCommand(0.2), new ParallelCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED), new ShootAmp(shooter)).withTimeout(1.5))); // waitcommand was 0.7
    operatorController.a().whileFalse(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_SAFE));

    //CLIMB
    operatorController.leftBumper().onTrue(new SequentialCommandGroup(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_UP), new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.changeStatus(), leftClimb), new InstantCommand(() -> rightClimb.changeStatus(), rightClimb)
    )));
    operatorController.x().whileTrue(new ParallelCommandGroup(new LowerClimbUntilLatch(leftClimb), new LowerClimbUntilLatch(rightClimb)));
  }

  public Command getAutonomousCommand() {
    return autoChooserLOL.getSelected();
  }
}