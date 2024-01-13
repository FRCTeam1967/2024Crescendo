package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    private SwerveModulePosition m_internalState = new SwerveModulePosition();


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
        cancoderConfig.apply(ccdConfigs);

        //configure power
        TalonFXConfiguration powerConfig = new TalonFXConfiguration();
        var powerControllerConfig = powerController.getConfigurator();
        
        powerConfig.Slot0.kP = Constants.Swerve.POWER_kP; 
        powerConfig.Slot0.kI = Constants.Swerve.POWER_kI; 
        powerConfig.Slot0.kD = Constants.Swerve.POWER_kD; 

        powerConfig.Voltage.PeakForwardVoltage = 7; //should probably make these constants
        powerConfig.Voltage.PeakReverseVoltage =-7;//should probably make these constants

        powerControllerConfig.apply(powerConfig);

        //configure steer
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        var steerControllerConfig = steerController.getConfigurator();

        steerConfig.Slot0.kP = Constants.Swerve.STEER_kP; 
        steerConfig.Slot0.kI = Constants.Swerve.STEER_kI; 
        steerConfig.Slot0.kD = Constants.Swerve.STEER_kD; 

        steerConfig.Voltage.PeakForwardVoltage = 7; //should probably make these constants
        steerConfig.Voltage.PeakReverseVoltage =-7;//should probably make these constants

        steerConfig.Feedback.FeedbackRemoteSensorID = analogEncoder.getDeviceID(); 
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.Feedback.RotorToSensorRatio = Constants.Swerve.GEAR_RATIO;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        steerControllerConfig.apply(steerConfig);

        //print out initial positions
        steerController.getPosition().refresh();
        analogEncoder.getAbsolutePosition().refresh();

        System.out.println("FX Position: " + steerController.getPosition().toString());
        System.out.println("CANcoder Position: " + analogEncoder.getAbsolutePosition().toString());

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


    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
        if (delta > 180.0) {
            delta -= 360.0;
        } else if (delta < -180.0) {
            delta += 360.0;
        }

        double targetAngle_deg = currentAngle.getDegrees() + delta;

        double targetSpeed_mps = desiredState.speedMetersPerSecond;

        if (delta > 90.0) {
            targetSpeed_mps = -targetSpeed_mps;
            targetAngle_deg -= 180.0;
        } else if (delta < -90.0) {
            targetSpeed_mps = -targetSpeed_mps;
            targetAngle_deg += 180.0;
        }

        return new SwerveModuleState(targetSpeed_mps, Rotation2d.fromDegrees(targetAngle_deg));
    }

    public void setState(SwerveModuleState state) {

        //our old optimize state code (wcp cc)
        // state.angle = Rotation2d.fromDegrees((state.angle.getDegrees() + 360) % 360);
        // SwerveModuleState optimizedState = optimize(state, getState().angle);

        var optimized = optimize(state, m_internalState.angle);

        //our old version (wcp cc) - only backwards movement
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
        steerController.setControl(new PositionVoltage(angleToSetDeg, 0.0, false, 0.0, 0, false, false, false));
        double velocityToSet = optimized.speedMetersPerSecond / Constants.Swerve.RPM_TO_MPS;
        powerController.setControl(new VelocityVoltage(velocityToSet, 0.0, false, 0.0, 0, false, false, false));
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



