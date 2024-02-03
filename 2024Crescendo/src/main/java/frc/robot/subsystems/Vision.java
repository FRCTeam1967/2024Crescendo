// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Vision extends SubsystemBase {
  private NetworkTable limelightTable;
  private double xOffset;
  private boolean isInRange = false;
  private ShuffleboardTab tab;

  /** Creates a new LimelightNetworkTable. */
public Vision() {
  limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
}


public void configDashboard(ShuffleboardTab tab){
  //tab.addCamera("Limelight Camera", "m_limelight", "http://10.19.67.11:5800/");
  this.tab = tab;
  tab.addDouble("Vision xOffset", () -> limelightTable.getEntry("tx").getDouble(0.0));
  tab.addDouble("Vision yOffset", () -> limelightTable.getEntry("ty").getDouble(0.0));
}

public boolean checkAlignment(){
  if (xOffset < -Constants.Vision.DEGREE_ERROR && xOffset > Constants.Vision.DEGREE_ERROR){
    isInRange = false;
    tab.addBoolean("Vision Align", () -> isInRange);
    return isInRange;
  } else {
    isInRange = true;
    tab.addBoolean("Vision Align", () -> isInRange);
    return isInRange;
  }
}
@Override
public void periodic() {
  // This method will be called once per scheduler run
  //xOffset = limelightTable.getEntry("tx").getDouble(0.0);
  //yOffset = limelightTable.getEntry("ty").getDouble(0.0);
  checkAlignment();
}
}