package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {
  public final SwerveModule frontLeft, frontRight, backLeft, backRight;
  private final ADIS16470_IMU gyro;
  private SimGyro simGyro;

  private Pose2d pose;
  public final SwerveDriveOdometry odometry;
  private Field2d field = new Field2d();

  private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
  private boolean isInRange = false;

  private StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/Swerve/States", SwerveModuleState.struct).publish();
  private StructArrayPublisher<SwerveModuleState> desiredStatesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/Swerve/DesiredStates", SwerveModuleState.struct).publish();

  public Swerve() {
    if(Robot.isSimulation()){
      simulationInit();
    }
    ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");

    frontLeft = new SwerveModule("FrontLeft", Constants.Swerve.FL_POWER, Constants.Swerve.FL_STEER,
        Constants.Swerve.FL_ENCODER, driveTrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0));
    frontRight = new SwerveModule("FrontRight", Constants.Swerve.FR_POWER, Constants.Swerve.FR_STEER,
        Constants.Swerve.FR_ENCODER, driveTrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0));
    backLeft = new SwerveModule("BackLeft", Constants.Swerve.BL_POWER, Constants.Swerve.BL_STEER,
        Constants.Swerve.BL_ENCODER, driveTrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0));
    backRight = new SwerveModule("BackRight", Constants.Swerve.BR_POWER, Constants.Swerve.BR_STEER,
        Constants.Swerve.BR_ENCODER, driveTrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0));
    gyro = new ADIS16470_IMU();

    driveTrainTab.addDouble("Gyro Angle", () -> getRotation2d().getDegrees());
    // driveTrainTab.add("field", field).withSize(8, 5).withPosition(1, 1);

    odometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });

    // took autobuilder from pathplanner - might need to be used in the auto file
    // (driveRobotRelative not coded yet)
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.00, 0.0, 0.0), // Translation PID constants //TODO change const.
            new PIDConstants(Constants.Auto.kPThetaController, 0.0, 0.0), // Rotation PID constants
            3.0, // Max module speed, in m/s
            0.41309, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent())
            return alliance.get() == DriverStation.Alliance.Red;
          else
            return false;
        }, this);
    
    
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public Field2d getField() {
    return field;
  }

  /** @return Rotation2d object with desired angle based on degrees from gyro */
  public Rotation2d getRotation2d() {
    if(Robot.isSimulation()){
      return simGyro.getRotation2d(); //TODO finish
    }
    return Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()));
  }

  // public double getYaw() {
  //   return gyro.getAngle(gyro.getYawAxis());
  //   //TODO add gyro sim
  // }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  // TODO: finish
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    this.setModuleStates(moduleState);
  }
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }


  public static double signedSquare(double a) {
    if (a < 0)
      return -(a * a);
    else
      return a * a;
  }

  public double getEncoderPosition() {
    return frontRight.getEncoderPosition();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    setModuleStates(desiredStates,true);
  }


  public void setModuleStates(SwerveModuleState[] desiredStates,boolean optimize) {
    frontLeft.setState(desiredStates[0],optimize);
    frontRight.setState(desiredStates[1],optimize);
    backLeft.setState(desiredStates[2],optimize);
    backRight.setState(desiredStates[3],optimize);

    desiredStatesPublisher.set(desiredStates);
  }

  public Pose2d getPose() {
    return pose;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetGyro() {
    gyro.setGyroAngle(gyro.getYawAxis(), 0);
  }

  public double getGyro() {
    //TODO add sim gyro
    if(Robot.isSimulation()){
      return simGyro.getRotation2d().getDegrees();
    }
    return gyro.getAngle(gyro.getYawAxis());
  }

  public void defenseMode() {
    SwerveModuleState fLDefenseState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState fRDefenseState = new SwerveModuleState(0, Rotation2d.fromDegrees(135));
    SwerveModuleState bLDefenseState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState bRDefenseState = new SwerveModuleState(0, Rotation2d.fromDegrees(135));

    frontLeft.setState(fLDefenseState,true);
    frontRight.setState(fRDefenseState,true);
    backLeft.setState(bLDefenseState,true);
    backRight.setState(bRDefenseState,true);
    System.out.println("lessgooo");

    frontLeft.brakeMode();
    frontRight.brakeMode();
    backLeft.brakeMode();
    backRight.brakeMode();
  }

  public void setNeutralMode(boolean brake) {
    if (brake) {
      frontLeft.brakeMode();
      frontRight.brakeMode();
      backLeft.brakeMode();
      backRight.brakeMode();
    } else {
      frontLeft.coastMode();
      frontRight.coastMode();
      backLeft.coastMode();
      backRight.coastMode();
    }
  }

  public void setGyroAngle(double degrees) {
    gyro.setGyroAngleZ(degrees);
  }

  public void configDashboard(ShuffleboardTab tab) {
    tab.addDouble("Power Encoder Position", () -> getEncoderPosition());
    tab.addDouble("pose position x", () -> getPose().getX());
    tab.addDouble("pose position y", () -> getPose().getY());
    tab.addDouble("Gyro Yaw Axis", () -> getGyro());
    tab.addBoolean("isReached?", () -> isInRange);

    tab.addDouble("Encoder Meters Moved", () -> getEncoderPosition() * Constants.Swerve.METERS_TO_ENC_COUNT);
    tab.addDouble("FR Distance Meters", () -> frontRight.getPosition().distanceMeters);
    tab.addDouble("FL Distance Meters", () -> frontLeft.getPosition().distanceMeters);
    tab.addDouble("BR Distance Meters", () -> backRight.getPosition().distanceMeters);
    tab.addDouble("BL Distance Meters", () -> backLeft.getPosition().distanceMeters);

    SmartDashboard.putNumber("Steer kS", 1);
    SmartDashboard.putNumber("Steer kV", 1);

    // tab.addDouble("steer reference", () ->
    // frontLeft.getSteerReference().getValueAsDouble())
    // .withWidget(BuiltInWidgets.kGraph);
    // tab.addDouble("steer rot?", () -> frontLeft.getSteerPosition())
    // .withWidget(BuiltInWidgets.kGraph);
  }

  @Override
  public void periodic() {
    getEncoderPosition();
    frontLeft.periodic();
     frontRight.periodic();
     backLeft.periodic();
     backRight.periodic();

     Rotation2d gyroAngle = getRotation2d(); 
     pose = odometry.update(gyroAngle, new SwerveModulePosition[] {
        frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()});
    SmartDashboard.putData("Field", field);
    field.setRobotPose(odometry.getPoseMeters());
    publisher.set(getModuleStates());
    }

    // TODO: is this used anywhere?
    // System.out.println(pose);

  public void simulationInit(){
    simGyro = new SimGyro();
  }
  public void simulationPeriodic() {
    frontLeft.simulationPeriodic();
    frontRight.simulationPeriodic();
    backLeft.simulationPeriodic();
    backRight.simulationPeriodic();

    simGyro.updateRotation(getRobotRelativeSpeeds().omegaRadiansPerSecond);

    odometry.update(simGyro.getRotation2d(), getModulePositions());

    field.setRobotPose(getPose());
    publisher.set(getModuleStates());
  }

  class SimGyro {
    private Rotation2d currRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currRotation;
    }

    public void updateRotation(double angularVelRadps) {
      currRotation = currRotation.plus(new Rotation2d(angularVelRadps * 0.02));
    }
  }

}
