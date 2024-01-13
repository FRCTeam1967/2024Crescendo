package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

// import statements


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;



public class SwerveModule {
    
    private TalonFX powerController;
    private TalonFX steerController;
    public CANcoder analogEncoder;

    private PositionVoltage m_angleSetter = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();
    private double m_driveRotationsPerMeter = 0;


    String name;

    public SwerveModule(String name, int powerIdx, int steerIdx, int encoderIdx, ShuffleboardLayout container) {
        this.name = name;

        // power controller set up
        powerController = new TalonFX(powerIdx, "Canivore");
        analogEncoder = new CANcoder(encoderIdx, "Canivore");

        var powerControllerConfig = powerController.getConfigurator();

        powerControllerConfig.apply(new TalonFXConfiguration());

        // set up pid
        var powerPidConfig = new Slot0Configs();
        powerPidConfig.kP = Constants.Swerve.POWER_kP; 
        powerPidConfig.kI = Constants.Swerve.POWER_kI; 
        powerPidConfig.kD = Constants.Swerve.POWER_kD; 

        powerControllerConfig.apply(powerPidConfig);

        // set max output range
        var voltageConfig = new VoltageConfigs();
        voltageConfig.PeakForwardVoltage = 7; //should probably make these constants
        voltageConfig.PeakReverseVoltage =-7;//should probably make these constants

        powerControllerConfig.apply(voltageConfig);
        
        // steer controller set up
        steerController = new TalonFX(steerIdx, "Canivore");

        var steerControllerConfig = steerController.getConfigurator();

        steerControllerConfig.apply(new TalonFXConfiguration());

        // set up pid
        var steerPidConfig = new Slot0Configs();
        steerPidConfig.kP = Constants.Swerve.STEER_kP; 
        steerPidConfig.kI = Constants.Swerve.STEER_kI; 
        steerPidConfig.kD = Constants.Swerve.STEER_kD; 

        steerControllerConfig.apply(steerPidConfig);

        // set max output range
        var steerVoltageConfig = new VoltageConfigs();
        steerVoltageConfig.PeakForwardVoltage = 7; //should probably make these constants
        steerVoltageConfig.PeakReverseVoltage =-7; //should probably make these constants

        steerControllerConfig.apply(steerVoltageConfig);

        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        analogEncoder.getConfigurator().apply(cancoderConfigs);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.FeedbackRemoteSensorID = analogEncoder.getDeviceID(); 
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1;
        config.Feedback.RotorToSensorRatio = Constants.Swerve.GEAR_RATIO;
    
        steerController.getConfigurator().apply(config);

        steerController.getPosition().refresh();
        analogEncoder.getAbsolutePosition().refresh();

        System.out.println("FX Position: " + steerController.getPosition().toString());
        System.out.println("CANcoder Position: " + analogEncoder.getAbsolutePosition().toString());

        //set wrapping config
        var wrappingConfig = new ClosedLoopGeneralConfigs();
        wrappingConfig.ContinuousWrap = true;

        steerControllerConfig.apply(wrappingConfig);

        powerController.stopMotor();
        steerController.stopMotor();

        addDashboardEntries(container);
    }
   
    //check to make sure steerController.getPosition will give us the angle?
    public SwerveModuleState getState() {
        return new SwerveModuleState(powerController.getPosition().getValue()*Constants.Swerve.RPM_TO_MPS,
        Rotation2d.fromDegrees(steerController.getPosition().getValue()));
    }

    //check to make sure this is actually getting the position of the swerve module (meters of pwr, angle of wheel)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            powerController.getPosition().getValue()*Constants.Swerve.MK4I_L1_REV_TO_METERS, getState().angle);
    }


    //
    private void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("Encoder Position in Rotations", () -> analogEncoder.getAbsolutePosition().getValueAsDouble());
        container.addNumber("Falcon Position in Rotations", () -> steerController.getPosition().getValueAsDouble());
        container.addNumber("Current Velocity", () -> this.getState().speedMetersPerSecond);
    }


    // not needed if continuous wrapping works
    // public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
    //     double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
    //     if (delta > 180.0) {
    //         delta -= 360.0;
    //     } else if (delta < -180.0) {
    //         delta += 360.0;
    //     }

    //     double targetAngle_deg = currentAngle.getDegrees() + delta;

    //     double targetSpeed_mps = desiredState.speedMetersPerSecond;

    //     if (delta > 90.0) {
    //         targetSpeed_mps = -targetSpeed_mps;
    //         targetAngle_deg -= 180.0;
    //     } else if (delta < -90.0) {
    //         targetSpeed_mps = -targetSpeed_mps;
    //         targetAngle_deg += 180.0;
    //     }

    //     return new SwerveModuleState(targetSpeed_mps, Rotation2d.fromDegrees(targetAngle_deg));
    // }

    public void setState(SwerveModuleState state) {

        // state.angle = Rotation2d.fromDegrees((state.angle.getDegrees() + 360) % 360);
        // SwerveModuleState optimizedState = optimize(state, getState().angle);

        //if no optimize: just use state as optimized state

        var optimized = SwerveModuleState.optimize(state, m_internalState.angle);

        //our version
        // create a velocity closed-loop request, voltage output, slot 0 configs
        //final VelocityVoltage setPwrRef = new VelocityVoltage(optimizedState.speedMetersPerSecond / Constants.Swerve.RPM_TO_MPS);
        // set velocity to 8 rps, add 0.5 V to overcome gravity
        //powerController.setControl(setPwrRef.withVelocity(optimizedState.speedMetersPerSecond / Constants.Swerve.RPM_TO_MPS));

        // create a position closed-loop request, voltage output, slot 0 configs
        //final PositionVoltage setSteerRef = new PositionVoltage(optimizedState.angle.getDegrees());
        // set position to 10 rotations
        //steerController.setControl(setSteerRef.withPosition(optimizedState.angle.getDegrees()));

        //stupid version
        // powerController.set(optimized.speedMetersPerSecond / Constants.Swerve.RPM_TO_MPS);
        // steerController.setPosition(optimized.angle.getDegrees());

        //ctre's version
        double angleToSetDeg = optimized.angle.getRotations();
        steerController.setControl(m_angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        powerController.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    public void stop() {
        powerController.stopMotor();
        steerController.stopMotor();
    }

    public void brakeMode() {

        var powerControllerConfig = powerController.getConfigurator();
        var steerControllerConfig = powerController.getConfigurator();
        
        var brakeModeConfig = new MotorOutputConfigs();
        brakeModeConfig.NeutralMode = NeutralModeValue.Brake;

        powerControllerConfig.apply(brakeModeConfig);
        steerControllerConfig.apply(brakeModeConfig);
        
    }

    public void coastMode() {

        var powerControllerConfig = powerController.getConfigurator();
        var steerControllerConfig = powerController.getConfigurator();
        
        var coastModeConfig = new MotorOutputConfigs();
        coastModeConfig.NeutralMode = NeutralModeValue.Coast;

        powerControllerConfig.apply(coastModeConfig);
        steerControllerConfig.apply(coastModeConfig);
        
    }
    
    public void periodic() {
        
    }
    
}



