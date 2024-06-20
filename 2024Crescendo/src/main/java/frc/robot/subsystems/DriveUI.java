// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// driverController.start().onTrue(new InstantCommand(() -> swerve.resetGyro(), swerve));
    // //driverController.x().onTrue(new InstantCommand(() -> swerve.defenseMode(), swerve)); 
    // driverController.a().onTrue(new AmpReverse(swerve, redAlliance));
    
    // driverController.b().onTrue(new VisionAlign(swerve, vision));

    // driverController.y().onTrue(new VisionAlignZ(swerve, vision));

    // driverController.leftTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->0));
    // //adjust for blue alliance
    // driverController.rightTrigger().whileTrue(new WallSnapDrive(swerve, () -> -driverController.getRawAxis(1), () -> -driverController.getRawAxis(0), ()->270));
    
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class DriveUI extends SubsystemBase {
  private final CommandXboxController driverController;
  public boolean isRed;
  private int driveX = 0; // Left X axis
  private int driveY = 1; // Left Y axis
  private int driveR = 4; // Left X axis

  private int driveAutoResumeIndex = 6; // R Bumper
  private int driveSnapIndex = 4; // Y Button

  private int runIntakeIndex = 5; // L Bumper
  private int interruptIntakeIndex = 7; // Back 

  private double maxSpeed = 3.9; //meters per sec
  private double maxRotation = Math.PI; //raidans per sec
  /** Creates a new DriveUI. */
  public DriveUI(String name, int port) {
    setName(name);
    driverController = new CommandXboxController(port);
    
    String osName = System.getProperty("os.name").toLowerCase();

    if(Robot.isSimulation()){
      if(osName.indexOf("mac") >= 0){
        driveR = 2;
        driveAutoResumeIndex = 8;
        driveSnapIndex = 5;
        runIntakeIndex = 7;
        interruptIntakeIndex = 11;
      }
    }
  }

  public double cleanScaleInput(double input, double scale, double deadband, double max){
    double scaleX = input * Math.abs(input) * scale;
    return MathUtil.applyDeadband(scaleX, deadband, max);
  }
  
  public double cleanScaleInput(double input, double scale, double max){
    return cleanScaleInput(input, scale, 0.05, max);
  }

  public boolean getAlliance(){
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      isRed = alliance.get() == DriverStation.Alliance.Red;
    } else {
      isRed = false;
    }
    return isRed;
  }

  public double getX(){
    double x = driverController.getRawAxis(driveX);
    x = cleanScaleInput(x, maxSpeed, maxSpeed);
    return x;
  }

  public double getY(){
    double y = driverController.getRawAxis(driveY);
    y = cleanScaleInput(y, maxSpeed, maxSpeed);
    return y;
  }

  public double getR(){
    double r = driverController.getRawAxis(driveR);
    r = cleanScaleInput(r, maxRotation, maxRotation);
    return r;
  }

  public Trigger getDriveSnapIndex(){
    return driverController.button(driveSnapIndex);
  }

  public boolean resumeDriveAuto(){
    boolean resumeAuto = driverController.button(driveAutoResumeIndex).getAsBoolean();
    return resumeAuto;
  }

  public boolean uninterruptDrive(){
    return(getX() == 0 && getY() == 0 && getR() == 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
