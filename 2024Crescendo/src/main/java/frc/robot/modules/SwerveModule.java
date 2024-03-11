package frc.robot.modules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    private TalonFX steerController;
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

        if (name == "BackLeft") {
            tuningTab.addNumber("YES Current Angle", () -> Currangle);
            tuningTab.addNumber("YES Desired", () -> desiredd);
            tuningTab.addNumber("YES Delta", () -> deltaa);
            tuningTab.addNumber("YES Optimized Angle", () -> optimizedAngle);

            tuningTab.addNumber("NO Current Angle", () -> NOCurrangle);
            tuningTab.addNumber("NO Desired", () -> NOdesiredd);
            tuningTab.addNumber("NO Delta", () -> NOdeltaa);
            tuningTab.addNumber("NO Optimized Angle", () -> NOoptimizedAngle);
        }

        // instantiate
        powerController = new TalonFX(powerIdx, "Canivore");
        steerController = new TalonFX(steerIdx, "Canivore");
        analogEncoder = new CANcoder(encoderIdx, "Canivore");

        //configure cancoder
        CANcoderConfiguration ccdConfigs = new CANcoderConfiguration();
        var cancoderConfig = analogEncoder.getConfigurator();

        // MDS: P2: Shouldn't we also be setting this to have SensorDirectionValue.CounterClockwise_Positive? 
        // Maybe this is working without that because the steer motor is getting uninverted in brake/coast mode below?
        // But even though those offset each other, that probably screws up left/right with respect to PathPlanner?
        ccdConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        // MDS: P3: These should reall go into Constants, and you should be able to pick between the two robots
        // using a DigitalIO. 
        if (name == "FrontLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = -133.857/360;
        }

        if (name == "FrontRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = 61.5234/360;
        }

        if (name == "BackLeft") {
            ccdConfigs.MagnetSensor.MagnetOffset = 113.203/360;
        }

        if (name == "BackRight") {
            ccdConfigs.MagnetSensor.MagnetOffset = -19.5996/360;
        }


        // if (name == "FrontLeft") {
        //     ccdConfigs.MagnetSensor.MagnetOffset = 171.738281/360;
        // }

        // if (name == "FrontRight") {
        //     ccdConfigs.MagnetSensor.MagnetOffset = -60.820313/360;
        // }

        // if (name == "BackLeft") {
        //     ccdConfigs.MagnetSensor.MagnetOffset = 122.695313/360;
        // }

        // if (name == "BackRight") {
        //     ccdConfigs.MagnetSensor.MagnetOffset = -153.808484/360;
        // }

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

        // MDS: P4: When we're trying to figure out odometry, we might want to try getting rid of this.
        // You have to do math anyway to convert wheel revolutions to meters, so you're not really 
        // avoiding any math, and you're trusting that this value is used consistently by the Phoenix 6.
        // Something to try if, when other issues are fixed, we still have problems. Of course, you'll 
        // need to change your MK4I_L1_REV_TO_METERS constant if you do this to include the gear ratio.
        // I'd only try this if the other issues don't solve the problem. AFAIK, what you're doing
        // *should* work.
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

        // MDS: P3: This is kind of slow because this is motor RPS, not wheel RPS. At a steer ration of ~21, 
        // this is only about 51 degrees/second of wheel rotation if I did the math right.  I'd think we'd 
        // want the wheel to be able to do a complete 360 in a second? Even that's kind of slow.
        steerConfig.MotionMagic.MotionMagicCruiseVelocity = 3; //rps (4)
        steerConfig.MotionMagic.MotionMagicAcceleration = 10; //rps/s
        steerConfig.MotionMagic.MotionMagicJerk = 0; //rps/s/s

        steerConfig.Feedback.SensorToMechanismRatio = 1;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = 40;

        steerConfig.Feedback.FeedbackRemoteSensorID = analogEncoder.getDeviceID(); 
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // MDS: P3: This is already set above.
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
        // MDS: P1: This is wrong. The first argument is a velocity, but we're passing a distance. It should probably be:
        // powerController.getVelocity().getValueAsDouble() * <wheel circumference> /* getVelocity() will be in wheel rotations/sec */
        // This probably confuses PathPlanner!
        return new SwerveModuleState(powerController.getPosition().getValue()*Constants.Swerve.RPM_TO_MPS,
        Rotation2d.fromRotations(steerController.getPosition().getValue()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            powerController.getPosition().getValue()*Constants.Swerve.MK4I_L1_REV_TO_METERS, getState().angle);
    }

    public double getEncoderPosition() {
        return powerController.getPosition().getValueAsDouble();
    }

    private void addDashboardEntries(ShuffleboardContainer container) {
        container.addNumber("Encoder Position in Degrees", () -> analogEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        // MDS: P3: This is correct, but parentheses would make it more clear that the mod operation happens last.
        container.addNumber("Falcon Position in Rotations", () -> steerController.getPosition().getValueAsDouble() * 360 % 360);

        // MDS: P4: Nothing to change here, but surely this value has been wrong on the dashboard given getState()'s implementation? After
        // moving the velocity wouldn't have returned to zero when the robot stopped?
        container.addNumber("Current Velocity", () -> this.getState().speedMetersPerSecond);

    }

    public SwerveModuleState optimize (SwerveModuleState desiredState, Rotation2d currentAngle) {
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
        
        // MDS: P2: I would think this is messing with our odometry. We're telling the motor to do the opposite of what SwerveModuleState says, 
        // so odometry probably thinks we're going backward when we're going forward. We shold fix this, but figure out why we're needing to
        // invert this here. I suspect we have a bug somewhere else.
        powerController.setControl(new VelocityVoltage(-velocityToSet/Constants.Swerve.WHEEL_CIRCUMFERENCE, 0.0, false, 0.0, 0, false, false, false));
    

    }

    
    public void stop() {
        powerController.stopMotor();
        steerController.stopMotor();
    }

    public void brakeMode() {

        // MDS: P1: You probably meant to grab the configurator for steerControllerConfig from the steerController!
        var powerControllerConfig = powerController.getConfigurator();
        var steerControllerConfig = powerController.getConfigurator();
        
        // MDS: P1: Because you're getting a new MotorOutputConfigs object and not reading it from the motor, we're blowing away
        // any other non-default configuration you might have applied at init-time in this config group. You should make sure that's
        // what you want to do. You might want to call <motor>.setNeutralMode() instead. That reads the value from the motor, and only
        // makes the one change. Otherwise, make sure you don't care that you're resetting back to factory default the following:
        // * Motor inversion!! <-- This seems like one you really don't want to be resetting
        // * Duty cycle stuff
        //
        // Looks like we set the steer motor to be inverted clockwise, but this is probably resetting it to counter-clockwise, though since
        // we're using power's configurator object on the steer controller, I'm not sure what happens. But fixing this might require another
        // change elsewhere because we're probably compensating for the incorrect inversion somewhere else.
        var brakeModeConfig = new MotorOutputConfigs();
        brakeModeConfig.NeutralMode = NeutralModeValue.Brake;

        powerControllerConfig.apply(brakeModeConfig);
        steerControllerConfig.apply(brakeModeConfig);
        
    }

    public void coastMode() {

        // MDS: P1: Same comments in brakeMode() apply here. Wrong configurator object. Resetting the inversion.
        var powerControllerConfig = powerController.getConfigurator();
        var steerControllerConfig = powerController.getConfigurator();
        
        var coastModeConfig = new MotorOutputConfigs();
        coastModeConfig.NeutralMode = NeutralModeValue.Coast;

        powerControllerConfig.apply(coastModeConfig);
        steerControllerConfig.apply(coastModeConfig);
        
    }

    // MDS: P3: This is kind of weird. Why don't we do this in the Swerve subsystem where it can grab all the states at once?
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



