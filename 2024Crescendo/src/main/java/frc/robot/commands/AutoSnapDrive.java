// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DriveUI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoSnapDrive extends Command {
  private final Swerve swerve;
  private final DriveUI driveUI;

  private boolean aim;
  private Translation2d speakerPos;
  private double rotationThreshold;
  private double rotationLimit;

  private SlewRateLimiter rLimiter;
  private FieldObject2d aimPos;

  private boolean alliance; // Red - true, Blue - false (based off driver UI)

  /** Creates a new AutoSnapDrive. */
  public AutoSnapDrive(Swerve swerve, DriveUI driveUI, double rotationThreshold) {
    this.swerve = swerve;
    this.driveUI = driveUI;
    this.rotationThreshold = rotationThreshold;
    aim = true;
    rotationLimit = 3 * Math.PI;
    rLimiter = new SlewRateLimiter(rotationLimit);
    updateAlliance();
    addRequirements(swerve);
    addRequirements(driveUI);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void updateAlliance() {
    alliance = driveUI.getAlliance();
    if (alliance) {
      speakerPos = new Translation2d(16.3, 5.45); // Red alliance center of speaker
      System.out.print("Red");
    } else {
      speakerPos = new Translation2d(0.25, 5.5); // Blue alliance center of speaker
      System.out.print("Blue");
    }
  }

  public double simpleProportionalController(double error, double gain, double min, double max) {
    return MathUtil.applyDeadband(error * gain, min * gain, max);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(aimPos == null){
      aimPos = swerve.getField().getObject("aimPos");
    }
    updateAlliance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed;
    double ySpeed;
    double rSpeed;
    if (driveUI.isRed) {
      xSpeed = driveUI.getY();
      ySpeed = driveUI.getX();
      rSpeed = -driveUI.getR();
    } else {
      xSpeed = -driveUI.getY();
      ySpeed = -driveUI.getX();
      rSpeed = -driveUI.getR();
    }

    if (driveUI.resumeDriveAuto()) {
      aim = true;
    }
    Pose2d currPose = swerve.getPose();
    Translation2d robotTranslate = currPose.getTranslation();

    if (robotTranslate.getDistance(speakerPos) < rotationThreshold){
      if (Math.abs(rSpeed) > 0.2) {
        aim = false;
        rLimiter.reset(rSpeed);
      } else if (aim) {
        double desiredRotSpeed;
        Translation2d relativePos = speakerPos.minus(robotTranslate);
        Rotation2d speakerDirection = relativePos.getAngle();
        Rotation2d rotateTo = speakerDirection.minus(currPose.getRotation());
        double rotationDegree = rotateTo.getDegrees();
        desiredRotSpeed = simpleProportionalController(rotationDegree, 1.0 / 20.0, 2, Math.PI);
        rSpeed = rLimiter.calculate(desiredRotSpeed);
        Transform2d aimPosTransform2d = new Transform2d(relativePos.getNorm(), 0, Rotation2d.fromDegrees(0));
        aimPos.setPose(currPose.plus(aimPosTransform2d));
      } else {
        aimPos.setPoses();
        rLimiter.reset(rSpeed);
      }
    } else {
      aim = true;
    }
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
    swerve.driveFieldRelative(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
