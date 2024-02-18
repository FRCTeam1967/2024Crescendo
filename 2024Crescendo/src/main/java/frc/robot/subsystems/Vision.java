package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Vision extends SubsystemBase {
  //https://readthedocs.org/projects/limelight/downloads/pdf/latest/
  private NetworkTable limelightTable;
  private double xOffset = -100000;
  private boolean isInRange = false;
  /** Creates a new Vision. */
  public Vision() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    updateValues();
  }

  public void updateValues() {
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    xOffset = tx.getDouble(0.0);

    SmartDashboard.putNumber("X Offset", xOffset);
  }

  public void configDashboard(ShuffleboardTab tab){
    tab.addCamera("Limelight Camera", "m_limelight", "http://10.19.67.11:5800/");

    tab.addDouble("Limelight xOffset", () -> limelightTable.getEntry("tx").getDouble(0.0));
    tab.addBoolean("In Range", ()->isInRange);
  }

  public void setVisionMode(boolean isVision){
    if (isVision){
        limelightTable.getEntry("pipeline").setNumber(0);
    } else {
        limelightTable.getEntry("pipeline").setNumber(1);
    }
  } 

  public void alignAngle(){
    updateValues();
    if (xOffset > -Constants.Vision.DEGREE_ERROR && xOffset < Constants.Vision.DEGREE_ERROR){
      isInRange = true;
      //tab.addBoolean("Vision Align1", () -> isInRange);
      SmartDashboard.putString("Range", "yes");
      //tab.addBoolean("Yes", ()->isInRange);
      //return true;
    } else {
      isInRange = false;
      //tab.addBoolean("Vision Align", () -> isInRange);
      SmartDashboard.putString("Range", "yes");
      //tab.addBoolean("Yes", ()->isInRange);
      //return false;
    }

    //tab.addBoolean("Range", ()->isInRange);
  }

  public boolean getIsInRange(){
    return isInRange;
  }

  public double getOffset() {
    return xOffset;
  }

  @Override
  public void periodic() {
    alignAngle();
    // This method will be called once per scheduler run
  }
}