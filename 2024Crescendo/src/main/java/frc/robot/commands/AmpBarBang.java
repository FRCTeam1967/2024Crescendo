// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpBar;

public class AmpBarBang extends Command {
  private AmpBar ampBar;
  private double startPos;
  private double targetPos;
  private double startReverse;
  private double startReversePos;
  private double stop;
  private double stopPos;
  private double speed;
  private double dirCoeff;
  private double tolerance;
  private double cruiseSpeed;
  private double minRPM;

  /**
   * Creates a new AmpBarBang.
   * @param ampBar       - AmpBar object
   * @param targetPos    - revolutions for desired position of AmpBar
   * @param startReverse - fraction of distance from starting point to the pos where you want to reverse motor power
   * @param stop         - fraction of distance to stop motor
   * @param tolerance    - stop condition error tolerance (in pos)
   * @param speed        - motor duty cycle to run (must always be positive)
   * @param cruiseSpeed - speed for cruise at
   * @param minRPM - motor minimum RPM during deceleration phase
   */
  public AmpBarBang(AmpBar ampBar, double targetPos, double startReverse, double stop, double tolerance, double speed, double cruiseSpeed,double minRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ampBar = ampBar;
    this.targetPos = targetPos;
    this.startReverse = startReverse;
    this.stop = stop;
    this.tolerance = tolerance;
    this.speed = speed;
    this.cruiseSpeed = cruiseSpeed;
    this.minRPM = minRPM;
    addRequirements(this.ampBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = ampBar.getArmPosition();
    startReversePos = startPos + (targetPos - startPos) * startReverse;
    stopPos = startPos + (targetPos - startPos) * stop;

    dirCoeff = (targetPos > startPos) ? 1 : -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dirCoeff > 0) { // Move in positive direction
      if (ampBar.getArmPosition() < startReversePos)
        ampBar.setDutyCycle(speed);
      else if ((ampBar.getArmPosition() >= startReversePos) && (ampBar.getArmPosition() < stopPos) && ampBar.getVelocity() >= minRPM)
        ampBar.setDutyCycle(-speed);
      else
        ampBar.setDutyCycle(cruiseSpeed); // Small number so motor don't break.
    } else { // Move in negative direction
      if (ampBar.getArmPosition() > startReversePos)
        ampBar.setDutyCycle(-speed);
      else if ((ampBar.getArmPosition() <= startReversePos) && (ampBar.getArmPosition() > stopPos) && ampBar.getVelocity() <= -minRPM)
        ampBar.setDutyCycle(speed);
      else
        ampBar.setDutyCycle(-cruiseSpeed); // Small number so motor don't break.
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampBar.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (dirCoeff > 0)
      return (ampBar.getArmPosition() > (targetPos - tolerance)) || ((ampBar.getVelocity() < 10.0)&&(ampBar.getArmPosition()>startReversePos));
    else
      return ampBar.getArmPosition() < (targetPos + tolerance) || ((ampBar.getVelocity() > -10.0)&&(ampBar.getArmPosition()<startReversePos));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    String name = getName();
    builder.addDoubleProperty("startPosition", () -> {return startPos;}, null);
    builder.addDoubleProperty("targetPosition", () -> {return targetPos;}, (var) -> {targetPos = var;});
    builder.addDoubleProperty("startReverse", () -> {return startReverse;}, (var) -> {startReverse = var;});
    builder.addDoubleProperty("stop", () -> {return stop;}, (var) -> {stop = var;});
    builder.addDoubleProperty("tolerance", () -> {return tolerance;}, (var) -> {tolerance = var;});
    builder.addDoubleProperty("speed", () -> {return speed;}, (var) -> {speed = var;});
    builder.addDoubleProperty("cruiseSpeed", () ->{return cruiseSpeed;},(var)->{cruiseSpeed=var;});
  }
}
