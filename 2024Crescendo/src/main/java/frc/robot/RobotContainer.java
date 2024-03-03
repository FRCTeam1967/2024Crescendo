





// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import edu.wpi.first.wpilibj2.command.*; //NOT WORKING
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import com.reduxrobotics.canand.CanandEventLoop;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Vision;

import frc.robot.commands.*;
import frc.robot.Constants.*;

public class RobotContainer {
  private final Climb leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID);
  private final Climb rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID);
  private final Pivot pivot = new Pivot();
  public final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();
  private boolean redAlliance;

  private Vision vision = new Vision();


  public static final String shootSit = "Shoot/Sit";
  public static final String leave = "Leave";
  public static final String shootLeave = "Shoot/Leave";





  private final CommandXboxController driverController = new CommandXboxController(Xbox.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(Xbox.OPERATOR_CONTROLLER_PORT);

  public ShuffleboardTab matchTab = Shuffleboard.getTab("Match");

  SendableChooser<String> autoPathChooser2 = new SendableChooser<String>(); 


  String autoPath;

  SendableChooser<DoubleSupplier> ampTimeout = new SendableChooser<DoubleSupplier>(); 

  public RobotContainer() {

    NamedCommands.registerCommand("pivotDown", new RunPivotIntakeBeam(pivot, intake, feeder, 0, 0).withTimeout(Constants.Auto.PIVOT_INTAKE_TIMEOUT));
    NamedCommands.registerCommand("pivotUp", new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE).withTimeout(Constants.Auto.PIVOT_UP_TIMEOUT));
    NamedCommands.registerCommand("shootSpeaker", new RunFeederShooter(shooter, feeder, Constants.Shooter.SPEAKER_TOP_VELOCITY, Constants.Shooter.SPEAKER_TOP_ACCELERATION, Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION).withTimeout(Constants.Auto.SHOOT_SPEAKER_TIMEOUT));

    resetSensors();
    // CanandEventLoop.getInstance();

    leftClimb.setToZero();
    rightClimb.setToZero();

    CanandEventLoop.getInstance();
    maintainPivotPosition();
    pivot.setBrakeMode();

    configureBindings();
    // maintainPosition();

    shooter.configDashboard(matchTab);
    feeder.configDashboard(matchTab);
    swerve.configDashboard(matchTab);
    leftClimb.configDashboard(matchTab);
    rightClimb.configDashboard(matchTab);

    /*autoPathChooser.setDefaultOption("Score/Intake", "scorePreloadIntakeMiddle");
    //autoPathChooser.addOption("Score/Score", "scorePreloadScoreMiddle");
    //autoPathChooser.addOption("Side/Hide", "ScoreAndHide");
    autoPathChooser.addOption("Sit/Shoot", "Shoot");
    autoPathChooser.addOption("Sit", "DoNothing");
    matchTab.add("Auto Path", autoPathChooser).withWidget(BuiltInWidgets.kComboBoxChooser);*/

    autoPathChooser2.setDefaultOption(shootSit, shootSit);
    autoPathChooser2.addOption(leave, leave);
    autoPathChooser2.addOption(shootLeave, shootLeave);
    matchTab.add("Auto Path Chooser 2", autoPathChooser2).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  public void setClimbToZero(){
    leftClimb.setToZero();
    rightClimb.setToZero();
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
    driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    driverController.button(3).onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve)); 
    driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()-> 0));
    driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()-> 270)); 

    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -driverController.getRawAxis(1),
      () -> -driverController.getRawAxis(0), () -> -driverController.getRawAxis(4), redAlliance));
    
    //INTAKE + PIVOT + FEEDER
    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileTrue(
      new SequentialCommandGroup(
        new RunPivotIntakeBeam(pivot, intake, feeder, Constants.Feeder.FEED_SPEED, Constants.Feeder.FEED_SPEED), 
        new ReverseBeamFeeder(feeder, Constants.Feeder.REVERSE_SPEED, Constants.Feeder.REVERSE_SPEED), 
        new RumbleController(driverController, operatorController, feeder).withTimeout(2)
      )
    );

    operatorController.start().whileTrue(new RunShooter(shooter, -Constants.Shooter.AMP_TOP_VELOCITY, -Constants.Shooter.AMP_TOP_ACCELERATION, -Constants.Shooter.AMP_BOTTOM_VELOCITY, -Constants.Shooter.AMP_BOTTOM_ACCELERATION));

    //driverController.a().onTrue(new VisionAlign(swerve, vision).withTimeout(3));

    operatorController.leftTrigger().or(operatorController.rightTrigger()).whileFalse(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE));
    operatorController.rightBumper().whileTrue(new ParallelCommandGroup(new RunIntake(intake, -Constants.Intake.INTAKE_ROLLER_SPEED), new RunFeeder(feeder, -Constants.Feeder.FEED_SPEED, -Constants.Feeder.FEED_SPEED)));
    
    intake.setDefaultCommand(new RunIntake(intake, 0));
    feeder.setDefaultCommand(new RunFeeder(feeder, 0, 0));
    
    //SHOOTER + FEEDER
    operatorController.y().whileTrue( //SPEAKER
      new RunFeederShooter(shooter, feeder, Constants.Shooter.SPEAKER_TOP_VELOCITY, Constants.Shooter.SPEAKER_TOP_ACCELERATION, Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION));
    // operatorController.a().onTrue( //AMP
    //   new SequentialCommandGroup(
    //     new AmpReverse(swerve, redAlliance),
    //     new ParallelCommandGroup(
    //       new RunFeeder(feeder, (Constants.Feeder.FEED_SPEED), (Constants.Feeder.FEED_SPEED)),
    //       new RunShooter(shooter, Constants.Shooter.AMP_TOP_VELOCITY, Constants.Shooter.AMP_TOP_ACCELERATION, Constants.Shooter.AMP_BOTTOM_VELOCITY, Constants.Shooter.AMP_BOTTOM_ACCELERATION)
    //     ).withTimeout(2)
    //   )
    // );

    operatorController.a().onTrue( //AMP
      new SequentialCommandGroup(
        new AmpReverseV2(swerve, ()-> 0.2 ,redAlliance),
        new ParallelCommandGroup(
          new RunFeeder(feeder, (Constants.Feeder.FEED_SPEED), (Constants.Feeder.FEED_SPEED)),
          new RunShooter(shooter, Constants.Shooter.AMP_TOP_VELOCITY, Constants.Shooter.AMP_TOP_ACCELERATION, Constants.Shooter.AMP_BOTTOM_VELOCITY, Constants.Shooter.AMP_BOTTOM_ACCELERATION)
        ).withTimeout(2)//2
      )
    );

    /*operatorController.start().whileTrue(new ParallelCommandGroup(
          new RunFeeder(feeder, (Constants.Feeder.FEED_SPEED), (Constants.Feeder.FEED_SPEED)),
          new RunShooter(shooter, Constants.Shooter.AMP_TOP_VELOCITY, Constants.Shooter.AMP_TOP_ACCELERATION, Constants.Shooter.AMP_BOTTOM_VELOCITY, Constants.Shooter.AMP_BOTTOM_ACCELERATION)
        ).withTimeout(3));*/
    
    // CLIMB
    /*operatorController.povUp().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.moveTo(Constants.Climb.TOP_ROTATIONS, false), leftClimb),
      new InstantCommand(() -> rightClimb.moveTo(Constants.Climb.TOP_ROTATIONS, false), rightClimb)));*/
    
    operatorController.povUp().onTrue(new ParallelCommandGroup(
      new RaiseClimb(leftClimb, leftClimb.isZero), new RaiseClimb(rightClimb, rightClimb.isZero)));

    operatorController.povDown().onTrue(new ParallelCommandGroup(
      new LowerClimbUntilLatch(leftClimb), new LowerClimbUntilLatch(rightClimb)));
    operatorController.x().onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.stop(), leftClimb), new InstantCommand(() -> rightClimb.stop(), rightClimb)
    ));
    
    leftClimb.setDefaultCommand(new InstantCommand(() -> leftClimb.stop(), leftClimb));
    rightClimb.setDefaultCommand(new InstantCommand(() -> rightClimb.stop(), rightClimb));
  }

  public Command getAutonomousCommand() {
    String auto = autoPathChooser2.getSelected();
   //return new PathPlannerAuto(autoPathChooser.getSelected());
   //return new InstantCommand(() -> {});
   
   //return ScoreSit();
   //return Leave();
   //return ScoreLeave();
  
  
   //  return new SequentialCommandGroup(
  //   new RunFeederShooter(shooter, feeder, Constants.Shooter.SPEAKER_TOP_VELOCITY, Constants.Shooter.SPEAKER_TOP_ACCELERATION, Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION).withTimeout(3),
  //   new SwerveDrive(swerve, ()-> 0.2, ()-> 0, ()-> 0, redAlliance).withTimeout(2),
  //   new RunPivotIntakeBeam(pivot, intake, feeder, Constants.Feeder.FEED_SPEED, Constants.Feeder.FEED_SPEED), 
  //   new ReverseBeamFeeder(feeder, Constants.Feeder.REVERSE_SPEED, Constants.Feeder.REVERSE_SPEED));

        switch (auto) {
          case shootSit:
            return ShootSit();
          case leave:
            return Leave();
          case shootLeave:
            return ShootLeave();
          default:
            return ShootSit();
        }

  }

  public Command ShootSit(){
    return new RunFeederShooter(shooter, feeder, Constants.Shooter.SPEAKER_TOP_VELOCITY, Constants.Shooter.SPEAKER_TOP_ACCELERATION, Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION).withTimeout(3);
  }

  public Command Leave(){
    return new SwerveDrive(swerve, ()->0.3, ()->0, ()->0, redAlliance).withTimeout(2);
  }

  public Command ShootLeave(){
     return new SequentialCommandGroup(new RunFeederShooter(shooter, feeder, 
              Constants.Shooter.SPEAKER_TOP_VELOCITY, Constants.Shooter.SPEAKER_TOP_ACCELERATION, 
              Constants.Shooter.SPEAKER_BOTTOM_VELOCITY, Constants.Shooter.SPEAKER_BOTTOM_ACCELERATION).withTimeout(3), 
            new SwerveDrive(swerve, ()->0.3, ()->0, ()->0, redAlliance).withTimeout(2));
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

  public void resetSwerveGyro(){
    swerve.resetGyro();
  }

}
