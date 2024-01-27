// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private HttpCamera limelightCamera;

  public Limelight(ShuffleboardTab limelightTab) {
    limelightCamera = new HttpCamera("Limelight Camera", "http://10.19.67.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    
    limelightCamera.setResolution(320, 240);
    limelightCamera.setFPS(30);

    CameraServer.startAutomaticCapture(limelightCamera);

    //limelightTab.add("limelight", limelightCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
