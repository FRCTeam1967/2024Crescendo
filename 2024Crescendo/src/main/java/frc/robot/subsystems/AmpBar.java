// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import frc.robot.Constants;
import frc.robot.Robot;

public class AmpBar extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private SparkPIDController pidController;
  private RelativeEncoder relativeEncoder;
  private Timer timer;
  private double neoRPM = 11000.0;
  // private TrapezoidProfile.Constraints motionProfile = new
  // TrapezoidProfile.Constraints(0.2 * neoRPS, 0.8 * neoRPS);

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(16.5, 18);
  private TrapezoidProfile profile = new TrapezoidProfile(motionProfile);
  public TrapezoidProfile.State setpoint = new TrapezoidProfile.State(Constants.AmpBar.AMP_SAFE, 0);
  public TrapezoidProfile.State goal = new TrapezoidProfile.State(Constants.AmpBar.AMP_SAFE, 0);

  public double revsToMove;

  // Variables for simulation
  private SimDeviceSim simMotor;
  private SimDouble simMotorVelocity;
  private DCMotorSim simAmpBar;
  private double simMotorRPS;
  private DCMotor simGearBox;
  private double simMotorVoltage;
  private PIDController simPID;
  private boolean simStop = false;
  private double targetRevs = 0;
  private double stopMargin = 0.2;

  /** Creates a new Pivot. */
  public AmpBar() {
    pivotMotor = new CANSparkMax(Constants.AmpBar.AMP_BAR_ID, MotorType.kBrushless);
    pivotMotor.setInverted(true);
    pidController = pivotMotor.getPIDController();
    pidController.setP(Constants.AmpBar.kP);
    pidController.setI(Constants.AmpBar.kI);
    pidController.setD(Constants.AmpBar.kD);
    pidController.setOutputRange(-0.2, 0.2);

    relativeEncoder = pivotMotor.getEncoder();

    pidController.setFeedbackDevice(relativeEncoder);
    timer = new Timer();

    // TODO: MAKE SURE MOTOR IS AT BARELY STARTING STAGE NOT FULLY BACK
    setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);

    if (Robot.isSimulation())
      simulationInit();

  }

  // public void resetEncoder({
  // relativeEncoder.reset
  // })

  /** Stops pivot motor */
  public void stop() {
    pivotMotor.stopMotor();
    simStop = true;
  }

  public double getPosition() {
    return relativeEncoder.getPosition();
  }

  public void runSecond() {
    timer.start();
    while (timer.get() <= 2) {
      pivotMotor.set(-0.2);
    }
    pivotMotor.set(0);
    timer.stop();
    timer.reset();
  }

  /**
   * Set encoder position to desired revolutions
   * 
   * @param rev
   */
  public void setPosition(double rev) {
    relativeEncoder.setPosition(rev);
  }

  /**
   * Sets motion profiling goal to desired revolutions
   * 
   * @param revolutions
   */
  public void moveTo(double revolutions) {
    goal = new TrapezoidProfile.State(revolutions, 0);
    simStop = false;

  }

  /**
   * @return whether profile has been finished
   */
  public boolean isReached() {
    // return (profile.isFinished(profile.timeLeftUntil(goal.position)));
    var pos = getPosition();
    var target = goal.position * Constants.AmpBar.GEAR_RATIO;

    if (target > 1.5)
      return pos > (target - stopMargin);
    else
      return pos < (target + stopMargin);
    // return ((target+0.2 >= pos)&&(target-0.2<= pos));
  }

  /** Sets pivot motor to brake mode */
  public void setBrakeMode() {
    pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // setpoint = profile.calculate(Constants.Pivot.kD_TIME, setpoint, goal);
    // double revs = (setpoint.position) * Constants.AmpBar.GEAR_RATIO;
    targetRevs = goal.position * Constants.AmpBar.GEAR_RATIO;

    pidController.setReference(targetRevs, CANSparkBase.ControlType.kPosition);

    SmartDashboard.putNumber("Amp Rel Pos", relativeEncoder.getPosition());
    SmartDashboard.putNumber("Amp Set Point", setpoint.position);
    SmartDashboard.putNumber("Amp revs", targetRevs);
    SmartDashboard.putNumber("Amp Rel Pos Degrees",
        (relativeEncoder.getPosition() * 360) / Constants.AmpBar.GEAR_RATIO);
  }

  private REVPhysicsSim physicsSim;
  private SimDouble simMotorPosition;
  double simPreviousMotorModelPos = 0;
  double simBackLash = 0;
  double simMotorPos = 0;
  double simArmPos = 0;

  double simArmPosAbsMin = 0;
  double simArmPosAbsMax = 2.6;

  public void simulationInit() {
    simMotor = new SimDeviceSim("SPARK MAX [24]");
    simMotorVelocity = simMotor.getDouble("Velocity");
    simMotorPosition = simMotor.getDouble("Position");
    simGearBox = DCMotor.getNeo550(1);
    simAmpBar = new DCMotorSim(simGearBox, 1, 0.0001);
    physicsSim = REVPhysicsSim.getInstance();
    physicsSim.addSparkMax(pivotMotor, simGearBox);

    simPID = new PIDController(1, 0, 0);
    simPID.setP(pidController.getP());
    simPID.setI(pidController.getI());
    simPID.setD(pidController.getD());
  }

  @Override
  public void simulationPeriodic() {
    if (simStop)
      simMotorVoltage = 0;
    else
      simMotorVoltage = MathUtil.clamp(simPID.calculate(simMotorPos, targetRevs),
          pidController.getOutputMin(), pidController.getOutputMax()) * RobotController.getBatteryVoltage();

    simAmpBar.setInputVoltage(simMotorVoltage);
    simAmpBar.update(0.02);

    double deltaPos = simAmpBar.getAngularPositionRotations() - simPreviousMotorModelPos;
    double simMotorOutDelta = 0;

    simPreviousMotorModelPos = simAmpBar.getAngularPositionRotations();

    // Modeling gearbox backlash.
    // Backlash is modele as part of motor.
    double prevSimBackLash = simBackLash;
    double motorPosDelta = 0;
    simBackLash = simBackLash + deltaPos;
    if (simBackLash > 0.125) {
      simMotorOutDelta = simBackLash - 0.125;
      simBackLash = 0.125;
    } else if (simBackLash < -0.125) {
      simMotorOutDelta = simBackLash - (-0.125);
      simBackLash = -0.125;
    } else {
      simMotorOutDelta = 0;
    }
    // Motor phase delta starts with motor movement under backlash.
    motorPosDelta = simBackLash - prevSimBackLash;

    // Modeling motor and arm phase change
    // The arm phase change is basically the true position of "arm" on the bot.
    double prevSimArmPos = simArmPos;
    double prevSimMotorPos = simMotorPos;
    double excessMotorPos = 0;

    simArmPos = simArmPos + simMotorOutDelta;
    if (simArmPos > simArmPosAbsMax) {
      excessMotorPos = simArmPos - simArmPosAbsMax;
      simArmPos = simArmPosAbsMax;
    } else if (simArmPos < simArmPosAbsMin) {
      excessMotorPos = simArmPos - simArmPosAbsMin;
      simArmPos = simArmPosAbsMin;
    } else {
      excessMotorPos = 0;
    }

    // Motor phase delta is basically arm phase delta + motor shaft to drive gearbox
    // slip .
    // The slip is guestimate to be 0.1 of excessMotorPos
    motorPosDelta = motorPosDelta + (simArmPos - prevSimArmPos) + excessMotorPos * 0.1;
    simMotorPos = simMotorPos + motorPosDelta;

    simMotorVelocity.set(((simMotorPos - prevSimMotorPos) / 0.002) * 60);
    simMotorPosition.set(simMotorPos);

    //physicsSim.run();
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    String name = getName();
    builder.addDoubleProperty("ampBarDesireRotation", () -> (goal.position), null);
    builder.addDoubleProperty("motorDesireVelocity", () -> pivotMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("motorCurrent", () -> pivotMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("motorVelocity", () -> relativeEncoder.getVelocity(), null);
    builder.addDoubleProperty("motorPosition", () -> relativeEncoder.getPosition(), null);
    builder.addBooleanProperty("isFinished", this::isReached, null);
    builder.addDoubleProperty("stopMargin", () -> {
      return stopMargin;
    }, (var) -> {
      stopMargin = var;
    });
    builder.addBooleanProperty("simStop", () -> {
      return simStop;
    }, null);

    if (Robot.isSimulation()) {
      SmartDashboard.putData(name + "/PID", simPID);
      builder.addDoubleProperty("simMotorVoltage", () -> {
        return simMotorVoltage;
      }, null);

      builder.addDoubleProperty("simArmPos", () -> {
        return simArmPos;
      }, null);

      builder.addDoubleProperty("simMotorPos", () -> {
        return simMotorPos;
      }, null);

            builder.addDoubleProperty("simBackLash", () -> {
        return simBackLash;
      }, null);

      builder.addDoubleProperty("simModelPos", () -> simAmpBar.getAngularPositionRotations(), null);

    }
  }

}