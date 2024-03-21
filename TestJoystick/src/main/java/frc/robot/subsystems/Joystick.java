// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Joystick extends SubsystemBase {
  /** Creates a new Joystick. */
  public Joystick() {}

  public void updateValues(DoubleSupplier speed) {
    double deadbandedSpeed = MathUtil.applyDeadband(speed.getAsDouble(), Constants.OperatorConstants.DEADBAND); //TODO: test if values update properly

    SmartDashboard.putNumber("speed",speed.getAsDouble());
    SmartDashboard.putNumber("deadbandedSpeed",deadbandedSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
