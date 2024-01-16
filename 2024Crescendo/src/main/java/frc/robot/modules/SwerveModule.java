package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

// import statements
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants;
import frc.robot.commands.SwerveDrive;



public class SwerveModule {
    
    private TalonFX powerController;
    private TalonFX steerController;
    public CANcoder analogEncoder;

    private SwerveModuleState initialState;
    private MotionMagicVoltage positionRequest;
    private MotionMagicVelocityVoltage velocityRequest;

    private double angleInRot = 0.0;
    private double lastAngleRot = 0.0;
    private static double lastAngleDeg = 0.0;


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

        if (name == "FrontLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = 170.156/360;
        }

        if (name == "FrontRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = -59.062/360;
        }

        if (name == "BackLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = 121.115/360;
        }

        if (name == "BackRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = -152.841/360;
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

        //powerConfig.MotionMagic.MotionMagicCruiseVelocity = 2; //rps (4)
        powerConfig.MotionMagic.MotionMagicAcceleration = 4; //rps/s
        powerConfig.MotionMagic.MotionMagicJerk = 9999; //rps/s/s

        powerConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.DRIVE_GEAR_RATIO ; // * Constants.Swerve.WHEEL_CIRCUMFERENCE
        powerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // powerConfig.Voltage.PeakForwardVoltage = 12; //should probably make these constants (commented out 1/15)
        // powerConfig.Voltage.PeakReverseVoltage = -12;//should probably make these constants

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

        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 2; //rps (4)
        steerConfig.MotionMagic.MotionMagicAcceleration = 3; //rps/s
        steerConfig.MotionMagic.MotionMagicJerk = 9999; //rps/s/s
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // steerConfig.Voltage.PeakForwardVoltage = 12; //should probably make these constants
        // steerConfig.Voltage.PeakReverseVoltage =-12;//should probably make these constants

        steerConfig.Feedback.FeedbackRemoteSensorID = analogEncoder.getDeviceID(); 
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = 1;
        //steerConfig.Feedback.RotorToSensorRatio = Constants.Swerve.STEER_GEAR_RATIO;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        steerControllerConfig.apply(steerConfig);

        //print out initial positions
        steerController.getPosition().refresh();
        analogEncoder.getAbsolutePosition().refresh();

        System.out.println("FX Position: " + steerController.getPosition().toString());
        System.out.println("CANcoder Position: " + analogEncoder.getAbsolutePosition().toString());

        //steerController.setControl(new PositionVoltage(0, 0.0, false, 0.0, 0, false, false, false));

        Rotation2d initialStartingAngle = new Rotation2d(0);
        initialState = new SwerveModuleState(0, initialStartingAngle);
        this.setState(initialState);

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
        container.addNumber("Encoder Position in Degrees", () -> analogEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        container.addNumber("Falcon Position in Rotations", () -> steerController.getPosition().getValueAsDouble() * 360 % 360);
        container.addNumber("Current Velocity", () -> this.getState().speedMetersPerSecond);

    }


    public static SwerveModuleState optimize(SwerveModuleState desiredState){
        double target = desiredState.angle.getDegrees();
        double current = lastAngleDeg;
        double delta = (target - current) % 180;
        double finalAngle = 0;
        if (delta > 90.0) {
            double addVar = 180 - delta;
            finalAngle = current - addVar;
        } else {
            finalAngle = current + delta;
        }
        
        System.out.println("Target Angle: " + target);
        System.out.println("Current: " + current);
        System.out.println("Final: " + finalAngle);

        double targetSpeed_mps = desiredState.speedMetersPerSecond;

        if ((target - current) >= 90 && (target- current) <= 180) {
            targetSpeed_mps = -targetSpeed_mps;
            System.out.println("Speed Switched");
            //targetAngle_deg -= 180.0;
        } else if ((target - current) <= -90 && (target- current) >= -180) {
            targetSpeed_mps = -targetSpeed_mps;
            System.out.println("Speed Switched");
            
            //targetAngle_deg += 180.0;
        }
        lastAngleDeg = finalAngle;
        return new SwerveModuleState(targetSpeed_mps, Rotation2d.fromDegrees(finalAngle));
    }

    public void setState(SwerveModuleState state) {

        //our optimize state code
        //var optimized = optimize(state);
        
       // SDS optimize code 
        var optimized = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(lastAngleDeg));
        lastAngleDeg = optimized.angle.getDegrees();

        angleInRot = Math.abs(optimized.speedMetersPerSecond) <= (Constants.Swerve.SWERVE_MAX_SPEED * 0.01) ? lastAngleRot : optimized.angle.getRotations() ;


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
        lastAngleRot = angleInRot;
        double velocityToSet = optimized.speedMetersPerSecond / Constants.Swerve.RPM_TO_MPS;
        
        //old
        //powerController.setControl(new VelocityVoltage(velocityToSet, 0.0, false, 0.0, 0, false, false, false));
        //powerController.set((optimized.speedMetersPerSecond / Constants.Swerve.RPM_TO_MPS)/20);
        //steerController.setControl(new PositionVoltage(angleInRot, 0.0, false, 0.3, 0, false, false, false));
        
        //motion magic
        velocityRequest = new MotionMagicVelocityVoltage(0);
        velocityRequest.Acceleration = 0;
        powerController.setControl(velocityRequest.withVelocity(velocityToSet/Constants.Swerve.WHEEL_CIRCUMFERENCE)); //in rps
       
        positionRequest = new MotionMagicVoltage(0);
        steerController.setControl(positionRequest.withPosition(angleInRot));
        


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



