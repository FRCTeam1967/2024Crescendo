package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
  //https://readthedocs.org/projects/limelight/downloads/pdf/latest/
  private NetworkTable limelightTable;
  private double xOffset = -100000;
  private double targetsDetected = -100;
  private boolean isInRange = false;
  private ShuffleboardTab tab;
  /** Creates a new Vision. */
  public Vision() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    updateValues();
  }

  public Vision(ShuffleboardTab visionTab) {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    updateValues();
    configDashboard(visionTab);
  }

  public void updateValues() {
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    NetworkTableEntry tv = limelightTable.getEntry("tv");
    
    xOffset = tx.getDouble(0.0);
    targetsDetected = tv.getDouble(0);
    SmartDashboard.putNumber("X  ", xOffset);
  }

  public void configDashboard(ShuffleboardTab tab){
    this.tab = tab;
    //tab.addCamera("Limelight Camera", "m_limelight", "http://10.19.67.11:5800/");
    tab.addBoolean("Limelight Camera1", () -> alignAngle());
    tab.addDouble("Limelight xOffset", () -> limelightTable.getEntry("tx").getDouble(0.0));
  }

  public void setVisionMode(boolean isVision){
    if (isVision){
        limelightTable.getEntry("pipeline").setNumber(0);
    } else {
        limelightTable.getEntry("pipeline").setNumber(1);
    }
  } 

  public boolean alignAngle(){
    updateValues();
    if (xOffset > -Constants.Vision.DEGREE_ERROR && xOffset < Constants.Vision.DEGREE_ERROR && targetsDetected>0.0){
      isInRange = true;
      //tab.addBoolean("Vision Align1", () -> isInRange);
      return isInRange;
    } else {
      isInRange = false;
      //tab.addBoolean("Vision Align", () -> isInRange);
      return isInRange;
    }

    //tab.addBoolean("Range", ()->isInRange);
  }

  public double getOffset() {
    return xOffset;
  }
  public double getTargetsDetected() {
    return targetsDetected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
