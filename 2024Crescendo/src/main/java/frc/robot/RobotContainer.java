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
import frc.robot.commands.AmpReverse;
import frc.robot.commands.LowerClimbUntilLatch;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.MoveAmpBar;
import frc.robot.commands.MovePivot;
import frc.robot.commands.ReverseBeamFeeder;
import frc.robot.commands.RumbleController;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunPivotIntakeBeam;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.VisionAlignZ;
import frc.robot.commands.WallSnapDrive;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final AmpBar ampBar = new AmpBar();
  private final Climb leftClimb = new Climb(Constants.Climb.LEFT_MOTOR_ID);
  private final Climb rightClimb = new Climb(Constants.Climb.RIGHT_MOTOR_ID);
  private final Pivot pivot = new Pivot();
  public final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();  
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();
  private final Vision vision = new Vision();
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

    vision.configDashboard(matchTab);
    shooter.configDashboard(matchTab);
    feeder.configDashboard(matchTab);
    swerve.configDashboard(matchTab);
    leftClimb.configDashboard(matchTab);
    rightClimb.configDashboard(matchTab);
    // ampBar.configDashboard(matchTab);

    autoChooserLOL = AutoBuilder.buildAutoChooser();
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
    //driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve)); 
    driverController.a().onTrue(new AmpReverse(swerve, redAlliance));
    
    driverController.b().onTrue(new VisionAlign(swerve, vision));

    driverController.y().onTrue(new VisionAlignZ(swerve, vision));

    driverController.povLeft().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    //adjust for blue alliance
    driverController.povRight().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));

     //INTAKE + PIVOT + FEEDER - testing with one joystick
    driverController.leftTrigger().or(driverController.rightTrigger()).whileTrue(new SequentialCommandGroup(
      new RunPivotIntakeBeam(pivot, intake, feeder),
      new ReverseBeamFeeder(feeder),
      new RumbleController(driverController, operatorController).withTimeout(2)
    ));
    
    driverController.leftTrigger().or(driverController.rightTrigger()).whileFalse(new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE));
    driverController.rightBumper().whileTrue(new ParallelCommandGroup(new RunIntake(intake, -Constants.Intake.INTAKE_ROLLER_SPEED), new RunFeeder(feeder, -Constants.Feeder.FEED_SPEED)));

    // driverController.povUp().whileTrue(new SwerveDrive(swerve, () -> 0.2, () -> 0, () -> 0));

    // driverController.povDown().whileTrue(new SwerveDrive(swerve, () -> -0.2, () -> 0, () -> 0));

    // driverController.povRight().whileTrue(new SwerveDrive(swerve, () -> 0, () -> 0.2, () -> 0));

    // driverController.povLeft().whileTrue(new SwerveDrive(swerve, () -> 0, () -> -0.2, () -> 0));
    
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
    // operatorController.a().onTrue(new SequentialCommandGroup(
    //   new AmpReverse(swerve,redAlliance),
    //   new ParallelCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED), new RunShooter(shooter, false)).withTimeout(1)
    // ));
    operatorController.start().whileTrue(new RunShooter(shooter, false));
    //CLIMB
    operatorController.x().whileTrue(new ParallelCommandGroup(new LowerClimbUntilLatch(leftClimb), new LowerClimbUntilLatch(rightClimb)));
    operatorController.leftBumper().onTrue(new SequentialCommandGroup(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_UP), new ParallelCommandGroup(
      new InstantCommand(() -> leftClimb.changeStatus(), leftClimb), new InstantCommand(() -> rightClimb.changeStatus(), rightClimb)
    )));

    operatorController.a().whileTrue(new SequentialCommandGroup(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_UP), new WaitCommand(0.7), new ParallelCommandGroup(new RunFeeder(feeder, Constants.Feeder.FEED_SPEED), new RunShooter(shooter, false)).withTimeout(1.5)));
    operatorController.a().whileFalse(new MoveAmpBar(ampBar, Constants.AmpBar.AMP_SAFE));
    // leftClimb.setDefaultCommand(new ManualClimb(() -> operatorController.getRightY(), leftClimb));
    // rightClimb.setDefaultCommand(new ManualClimb(() -> operatorController.getLeftY(), rightClimb));
  }

  public void resetSensors() {
    // swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));
    swerve.resetOdometry(new Pose2d(0.0, 0.0, swerve.getRotation2d()));

    swerve.frontLeft.resetEncoder();
    swerve.frontRight.resetEncoder();
    swerve.backLeft.resetEncoder();
    swerve.backRight.resetEncoder();
    swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
      swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    });;
    // swerve.odometry.update(swerve.getRotation2d(), new SwerveModulePosition[] {
    //   swerve.frontLeft.getPosition(), swerve.frontRight.getPosition(), swerve.backLeft.getPosition(), swerve.backRight.getPosition()
    // });;
  }

  public Command getAutonomousCommand() {
    return autoChooserLOL.getSelected();
    
    // switch (autoPath) {
    //   case BlueCenterDisrupt:
    //     return BlueCenterDisrupt();
    //   case RedCenterDisrupt:
    //     return RedCenterDisrupt();
    //   case redFourNote:
    //     return RedFourNote();
    //   case redThreeNote:
    //     return RedThreeNote();
    //   case blueThreeNote:
    //     return BlueThreeNote();
    //   case blueFourNote:
    //     return BlueFourNote();
    //   case twoNote:
    //     return TwoNote();
    //   case leave:
    //     return Leave();
    //   case frontShootSit:
    //     return FrontShootSit();
    //   case leftSideSit:
    //     return LeftSideSit();
    //   case rightSideSit:
    //     return RightSideSit();
    //   case leftSideLeave:
    //     return LeftSideLeave();
    //   case rightSideLeave:
    //     return RightSideLeave();
    //   case doNothing:
    //     return DoNothing();
    //   default:
    //     return FrontShootSit();
    // }
    //return BlueFourNote();
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

//   public Command FrontShootSit(){
//     return new ShootSpeaker(shooter, feeder).withTimeout(3);
//   }

//   public Command LeftSideSit(){
//     swerve.setGyroAngle(60); //idk the number yet
//     return new ShootSpeaker(shooter, feeder).withTimeout(3);
//   }

//   public Command RightSideSit(){
//     swerve.setGyroAngle(-60); //idk the number yet
//     return new ShootSpeaker(shooter, feeder).withTimeout(3);
//   }

//   //not leaving, doesn't work
//   public Command RightSideLeave(){
//     swerve.setGyroAngle(-60); //idk the number yet
//     return new SequentialCommandGroup(new ShootSpeaker(shooter, feeder).withTimeout(3), new SwerveDrive(swerve, ()->0.5, ()->0, ()->0).withTimeout(1.5));
//   }

//   //not leaving, doesn't work
//   public Command LeftSideLeave(){
//     swerve.setGyroAngle(60); //idk the number yet
//     return new SequentialCommandGroup(new ShootSpeaker(shooter, feeder).withTimeout(3), new SwerveDrive(swerve, ()->0.5, ()->0, ()->0).withTimeout(1.5));
//   }


//   public Command Leave(){
//     return new SwerveDrive(swerve, ()->0.5, ()->0, ()->0).withTimeout(1.5);
//   }

//   public Command DoNothing(){
//     return null;
//   }

//   public Command TwoNote(){
//     return new SequentialCommandGroup(
//       new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
//       new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
//       new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
//       new ShootSpeaker(shooter, feeder).withTimeout(1.5));
//   }

//   public Command BlueFourNote(){
//     return new SequentialCommandGroup(
//     new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.59, ()->0.715, ()->0)).withTimeout(1.3),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.59, ()->-0.715, ()->0)).withTimeout(1.34),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.569, ()->-0.656, ()->0)).withTimeout(1.32),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.569, ()->0.656, ()->0)).withTimeout(1.43),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5));
// }
// public Command RedFourNote(){
//   return new SequentialCommandGroup(
//     new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.59, ()->-0.715, ()->0)).withTimeout(1.3),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.59, ()->0.715, ()->0)).withTimeout(1.34),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.569, ()->0.656, ()->0)).withTimeout(1.32),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.569, ()->-0.656, ()->0)).withTimeout(1.43),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5));
// }

// public Command BlueCenterDisrupt(){
//   return new SequentialCommandGroup(
//     new SwerveDrive(swerve, ()->0.8, ()->0, ()->0).withTimeout(2), 
//     new SwerveDrive(swerve, ()->0.0, ()->0.8, ()->0).withTimeout(0.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(2.2)
//   );
// }

// public Command RedCenterDisrupt(){
//   return new SequentialCommandGroup(
//     new SwerveDrive(swerve, ()->0.8, ()->0, ()->0).withTimeout(2), 
//     new SwerveDrive(swerve, ()->0.0, ()->-0.8, ()->0).withTimeout(0.75),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(1.6)
//   );
// }

// public Command RedThreeNote(){
//   return new SequentialCommandGroup(
//     new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.615, ()->-0.7, ()->0)).withTimeout(1.3),
//     new RunPivotIntakeBeam(pivot, intake, feeder).withTimeout(0.2),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.615, ()->0.7, ()->0)).withTimeout(1.34),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5));
// }

// public Command BlueThreeNote(){
//   return new SequentialCommandGroup(
//     new ParallelCommandGroup(new MovePivot(pivot, Constants.Pivot.INTAKE_DOWN), new ShootSpeaker(shooter, feeder)).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ParallelCommandGroup(new SwerveDrive(swerve, ()->-0.8, ()->0, ()->0)).withTimeout(0.65),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->0.615, ()->0.7, ()->0)).withTimeout(1.3),
//     new RunPivotIntakeBeam(pivot, intake, feeder).withTimeout(0.2),
//     new ParallelCommandGroup(new RunPivotIntakeBeam(pivot, intake, feeder), new SwerveDrive(swerve, ()->-0.615, ()->-0.7, ()->0)).withTimeout(1.34),
//     new ShootSpeaker(shooter, feeder).withTimeout(1.5));
// }
}