package frc.robot.modules;

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
import com.ctre.phoenix6.configs.MotorOutputConfigs;

// import statements
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;



public class SwerveModule {
    
    private TalonFX powerController;
    public TalonFX steerController;
    public CANcoder analogEncoder;


    private SwerveModuleState initialState;

    private final StructArrayPublisher<SwerveModuleState> publisher;

    ShuffleboardTab tuningTab = Shuffleboard.getTab("Tuning");
    public double Currangle;
    public double deltaa;
    public double desiredd;
    public double optimizedAngle;
    public double NOCurrangle;
    public double NOdeltaa;
    public double NOdesiredd;
    public double NOoptimizedAngle;
    
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
            tuningTab.addNumber("FL Power Encoder", () -> powerController.getRotorPosition().getValueAsDouble());
        }

        if (name == "FrontRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.FR_OFFSET;
            tuningTab.addNumber("FR Power Encoder", () -> powerController.getRotorPosition().getValueAsDouble());
        }

        if (name == "BackLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.BL_OFFSET;
            tuningTab.addNumber("BL Power Encoder", () -> powerController.getRotorPosition().getValueAsDouble());
        }

        if (name == "BackRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = Constants.Swerve.BR_OFFSET;
            tuningTab.addNumber("BR Power Encoder", () -> powerController.getRotorPosition().getValueAsDouble());
        }

        if (name == "BackLeft") {
            tuningTab.addNumber("BL Power Motor Encoder", () -> powerController.getRotorPosition().getValueAsDouble());
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

        //powerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        powerConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO ; 

        powerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        powerConfig.CurrentLimits.StatorCurrentLimit = 40;

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

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 3; //rps (4)
        steerConfig.MotionMagic.MotionMagicAcceleration = 10; //rps/s
        steerConfig.MotionMagic.MotionMagicJerk = 0; //rps/s/s
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = 40;

        steerConfig.Feedback.FeedbackRemoteSensorID = analogEncoder.getDeviceID(); 
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = 1;

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
        Rotation2d.fromRotations(steerController.getPosition().getValue())); //maybe don't use getPosition() ??
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (powerController.getRotorPosition().getValueAsDouble()/Constants.Swerve.DRIVE_GEAR_RATIO)*Constants.Swerve.WHEEL_CIRCUMFERENCE, getState().angle);
    }

    public double getEncoderPosition() {
        return powerController.getRotorPosition().getValueAsDouble();
    }

    private void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("Encoder Position in Degrees", () -> analogEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        container.addNumber("Falcon Position in Rotations", () -> (steerController.getPosition().getValueAsDouble() * 360) % 360);
        container.addNumber("Current Velocity", () -> this.getState().speedMetersPerSecond);

    }

    public SwerveModuleState optimize (
    
        SwerveModuleState desiredState, Rotation2d currentAngle) {
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90.0) {
        if (this.name == "BackLeft") {
            Currangle = currentAngle.getDegrees();
            desiredd = desiredState.angle.getDegrees();
            deltaa = delta.getDegrees();
            optimizedAngle = desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)).getDegrees();
        }
        return new SwerveModuleState(
            -desiredState.speedMetersPerSecond,
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
      } else {
        if (this.name == "BackLeft") {
            NOCurrangle = currentAngle.getDegrees();
            NOdesiredd = desiredState.angle.getDegrees();
            NOdeltaa = delta.getDegrees();
            NOoptimizedAngle = desiredState.angle.getDegrees();
            
        }
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

        var powerControllerConfig = powerController.getConfigurator();
        var steerControllerConfig = steerController.getConfigurator();
        
        var brakeModeConfig = new MotorOutputConfigs();
        brakeModeConfig.NeutralMode = NeutralModeValue.Brake;

        powerControllerConfig.apply(brakeModeConfig);
        steerControllerConfig.apply(brakeModeConfig);
        
    }

    public void coastMode() {

        var powerControllerConfig = powerController.getConfigurator();
        var steerControllerConfig = steerController.getConfigurator();
        
        var coastModeConfig = new MotorOutputConfigs();
        coastModeConfig.NeutralMode = NeutralModeValue.Coast;

        powerControllerConfig.apply(coastModeConfig);
        steerControllerConfig.apply(coastModeConfig);
        
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



