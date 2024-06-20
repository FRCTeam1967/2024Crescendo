// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DriveUI;
import edu.wpi.first.math.geometry.Translation2d;

public class SnapToShoot extends Command {
  private final Swerve swerve;
  private final DriveUI driveUI;

  private List<Pose2d> shootingPoses;
  private double snapThreshold;
  private Pose2d targetPose;
  private double rotationLimit;

  private boolean isRedAlliance; // Red - true, Blue - false (based off driver UI)
  private boolean done;

  private Timer timer;
  /** Creates a new SnapToShoot. */
  public SnapToShoot(Swerve swerve,DriveUI driveUI, double snapThreshold){
    this.swerve = swerve;
    this.driveUI = driveUI;
    this.snapThreshold = snapThreshold;
    timer = new Timer();
    rotationLimit = 4 * Math.PI;
    addRequirements(swerve);
    addRequirements(driveUI);
    updateAlliance();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void updateAlliance() {
    isRedAlliance = driveUI.getAlliance();
    if (isRedAlliance) { // Red Allianc
      shootingPoses = new ArrayList<Pose2d>();
      shootingPoses.add(new Pose2d(15.8,6.83,Rotation2d.fromDegrees(-55)));
      shootingPoses.add(new Pose2d(15.1,5.5,Rotation2d.fromDegrees(0)));
      shootingPoses.add(new Pose2d(15.75,4.4,Rotation2d.fromDegrees(61.5)));
      System.out.print("Red");
    } else {
      shootingPoses = new ArrayList<Pose2d>();
      shootingPoses.add(new Pose2d(1.2, 6.3, Rotation2d.fromDegrees(-135)));
      shootingPoses.add(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180)));
      shootingPoses.add(new Pose2d(1.2, 4.75, Rotation2d.fromDegrees(135)));
      System.out.print("Blue");
    }
  }

  public double simpleProportionalController(double error, double gain, double min, double max) {
    return MathUtil.applyDeadband(error * gain, min * gain, max);
  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateAlliance();
    Pose2d robotPose = swerve.getPose();
    targetPose = robotPose.nearest(shootingPoses);
    done = false;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(targetPose != null){
      Pose2d robotPose = swerve.getPose();
     
      double distance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
      Translation2d translationOffset = targetPose.getTranslation().minus(robotPose.getTranslation());
      Rotation2d rotationOffset = targetPose.getRotation().minus(robotPose.getRotation());

      if(distance > snapThreshold){
        done = true;
      } else if(distance > 0.05| Math.abs(rotationOffset.getDegrees()) > 5.5){
        double xSpeed = simpleProportionalController(translationOffset.getX(), 3, 0.01, 1);
        double ySpeed = simpleProportionalController(translationOffset.getY(), 3, 0.01, 1);
        double rSpeed = simpleProportionalController(rotationOffset.getDegrees(), 1.0/5.0, 0, rotationLimit);
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        swerve.driveFieldRelative(chassisSpeeds);
      } else{
        done = true;
      }
    } else {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (done | timer.get() > 7.0 | !driveUI.uninterruptDrive());
  }
}
