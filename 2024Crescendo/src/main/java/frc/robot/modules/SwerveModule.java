package frc.robot.modules;

import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

// import statements
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;



public class SwerveModule {
    
    private TalonFX powerController;
    private TalonFX steerController;
    public CANcoder analogEncoder;


    private SwerveModuleState initialState;

    private final StructArrayPublisher<SwerveModuleState> publisher;

    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
   
    
    String name;

    public SwerveModule(String name, int powerIdx, int steerIdx, int encoderIdx, ShuffleboardLayout container) {
        this.name = name;
        // instantiate
        powerController = new TalonFX(powerIdx, "Canivore");
        steerController = new TalonFX(steerIdx, "Canivore");
        analogEncoder = new CANcoder(encoderIdx, "Canivore");

        //configure cancoder
        CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();
        var cancoderConfig = analogEncoder.getConfigurator();

        ccdConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        ccdConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;


        if (name == "FrontLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.FL_OFFSET;
        }

        if (name == "FrontRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.FR_OFFSET;
        }

        if (name == "BackLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.BL_OFFSET; 
        }

        if (name == "BackRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.BR_OFFSET; 
        }

        cancoderConfig.apply(ccdConfigs);

        //configure power
        TalonFXConfiguration powerConfig = new TalonFXConfiguration();
        var powerControllerConfig = powerController.getConfigurator();
        
        powerConfig.Slot0.kS = Constants.Swerve.POWER_kS; 
        powerConfig.Slot0.kV = Constants.Swerve.POWER_kV; 
        powerConfig.Slot0.kA = Constants.Swerve.POWER_kA; 
        powerConfig.Slot0.kP = Constants.Swerve.POWER_kP; 
        powerConfig.Slot0.kI = Constants.Swerve.POWER_kI; 
        powerConfig.Slot0.kD = Constants.Swerve.POWER_kD; 

        powerConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO ; 

        powerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        powerConfig.CurrentLimits.StatorCurrentLimit = 40;

        powerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        powerConfig.CurrentLimits.SupplyCurrentLimit = 40;

        powerControllerConfig.apply(powerConfig);

        //configure steer
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        var steerControllerConfig = steerController.getConfigurator();

        steerConfig.Slot0.kS = Constants.Swerve.STEER_kS;
        steerConfig.Slot0.kV = Constants.Swerve.STEER_kV;
        steerConfig.Slot0.kA = Constants.Swerve.STEER_kA;
        steerConfig.Slot0.kP = Constants.Swerve.STEER_kP; 
        steerConfig.Slot0.kI = Constants.Swerve.STEER_kI; 
        steerConfig.Slot0.kD = Constants.Swerve.STEER_kD; 

        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = 40;

        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = 40;

        steerConfig.Feedback.FeedbackRemoteSensorID = analogEncoder.getDeviceID(); 
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        steerControllerConfig.apply(steerConfig);

        steerController.getPosition().refresh();
        analogEncoder.getAbsolutePosition().refresh();

        powerController.setPosition(0);

        Rotation2d initialStartingAngle = new Rotation2d(0);
        initialState = new SwerveModuleState(0, initialStartingAngle);
        this.setState(initialState);

        powerController.stopMotor();
        steerController.stopMotor();

        addDashboardEntries(container);

        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    }
   
    public void resetEncoder() {
        powerController.setPosition(0);
    }
    
    //check to make sure steerController.getPosition will give us the angle?
    public SwerveModuleState getState() {
        return new SwerveModuleState(powerController.getVelocity().getValueAsDouble()*Constants.Swerve.WHEEL_CIRCUMFERENCE,
        Rotation2d.fromRotations(steerController.getPosition().getValue()));
    }
    //public double getSteerPosition(){
       //return steerController.getPosition().getValue();
    //}

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (powerController.getRotorPosition().getValueAsDouble()/Constants.Swerve.DRIVE_GEAR_RATIO)*Constants.Swerve.WHEEL_CIRCUMFERENCE, getState().angle);
    }

    // public SwerveModulePosition getSteerRotation() {
    //     return new SwerveModulePosition(
    //         (steerController.getRotorPosition().getValueAsDouble()/Constants.Swerve.DRIVE_GEAR_RATIO)*Constants.Swerve.WHEEL_CIRCUMFERENCE, getState().angle);
    // }

    public double getEncoderPosition() {
        return powerController.getRotorPosition().getValueAsDouble();
    }

    private void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("CCD Position in Degrees", () -> analogEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        container.addNumber("Pwr Encoder Count", () -> getEncoderPosition());
        container.addNumber("Str Position in Rotations", () -> steerController.getPosition().getValueAsDouble() * 360 % 360);
        container.addNumber("Current Velocity", () -> this.getState().speedMetersPerSecond);
        container.addNumber("Current Angle in Deg", () -> this.getState().angle.getDegrees());
        //container.addDouble("Get Position", () -> backlegetPosition().getValueAsDouble());
    }

    public SwerveModuleState optimize (SwerveModuleState desiredState, Rotation2d currentAngle) {
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90.0) {
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
      } else {
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    public void setState(SwerveModuleState state) {
        var optimized = optimize(state, (this.getState().angle));
        double velocityToSet = optimized.speedMetersPerSecond;
        steerController.setControl(new PositionVoltage(optimized.angle.getRotations(), 0.0, false, 0.0, 0, false, false, false));
        powerController.setControl(new VelocityVoltage(-velocityToSet/Constants.Swerve.WHEEL_CIRCUMFERENCE, 0.0, false, 0.0, 0, false, false, false));
    }

    public void stop() {
        powerController.stopMotor();
        steerController.stopMotor();
    }

    public void brakeMode() {
        powerController.setNeutralMode(NeutralModeValue.Brake);
        steerController.setNeutralMode(NeutralModeValue.Brake);
    }

    public void coastMode() {
        powerController.setNeutralMode(NeutralModeValue.Coast);
        steerController.setNeutralMode(NeutralModeValue.Coast);
    }

    public StatusSignal<Double> getSteerReference() {
        return steerController.getClosedLoopReference();
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] arr = new SwerveModuleState[4];
        if (this.name == "FrontLeft") {
            arr[0] = this.getState();
        }
        if (this.name == "FrontRight") {
            arr[1] = this.getState();
        }
        if (this.name == "BackLeft") {
            arr[2] = this.getState();
        }
        if (this.name == "BackRight") {
            arr[3] = this.getState();
        }
        return arr;
    }
   
    public void periodic() {
        publisher.set(getStates());

    }
    
}



