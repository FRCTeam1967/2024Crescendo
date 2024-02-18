package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
  //https://readthedocs.org/projects/limelight/downloads/pdf/latest/
  private NetworkTable limelightTable;
  private double xOffset = -100000;
  private boolean isInRange = false;

  /** Creates new Vision */
  public Vision() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    updateValues();
  }

  /** Update x offset value */
  public void updateValues() {
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    xOffset = tx.getDouble(0.0);
    SmartDashboard.putNumber("X Offset", xOffset);
  }

  /**
   * Displays limelight camera view, xOffset value, and boolean showing whether limelight is in range on Shuffleboard
   * @param tab - ShuffleboardTab to add values to
   */
  public void configDashboard(ShuffleboardTab tab){
    tab.addCamera("Limelight Camera", "m_limelight", "http://10.19.67.11:5800/");
    tab.addDouble("Limelight xOffset", () -> limelightTable.getEntry("tx").getDouble(0.0));
    tab.addBoolean("In Range", ()->isInRange);
  }

  /**
   * Changes pipeline
   * @param isVision - if true, look for AprilTags
   */
  public void setVisionMode(boolean isVision){
    if (isVision) limelightTable.getEntry("pipeline").setNumber(0);
    else limelightTable.getEntry("pipeline").setNumber(1);
  }

  /** updates value of isInRange */
  public void alignAngle(){
    updateValues();
    if (xOffset > -Constants.Vision.DEGREE_ERROR && xOffset < Constants.Vision.DEGREE_ERROR){
      isInRange = true;
      SmartDashboard.putString("Range", "yes");
    } else {
      isInRange = false;
      SmartDashboard.putString("Range", "yes");
    }
  }

  /** @return whether limelight is in range */
  public boolean getIsInRange(){
    return isInRange;
  }

  /** @return value of xOffset */
  public double getOffset() {
    return xOffset;
  }

  @Override
  public void periodic() {
    alignAngle();
  }
}