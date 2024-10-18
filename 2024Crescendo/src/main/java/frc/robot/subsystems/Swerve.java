package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class Swerve extends SubsystemBase{
  
  //swerve modules, pose estimator (odometry) object, gyro
  public SwerveModule frontLeft, frontRight, backLeft, backRight;
  public ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public final SwerveDrivePoseEstimator m_poseEstimator;

  //vision fields
  private Pose2d pose;
  private Field2d field = new Field2d();
  private Rotation2d gyroAngle;
  private SwerveModulePosition[] modulePositions;
  private Pose2d initialPoseMeters;

  //publishing to network table
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArrayPublisher publishField = table.getDoubleArrayTopic("robotPose").publish();
  private StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Pose2d", Pose2d.struct).publish();

  //extra
  private boolean isInRange = false;

  public Swerve() {
    ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");
    
    //gyro
    m_gyro = new ADIS16470_IMU();

    //pose estimator as odometry object + to get vision values
    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Swerve.SWERVE_DRIVE_KINEMATICS, 
      getRotation2d(), 
      getModulePositions(), 
      new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));

    //initializing actual swerve modules
    frontLeft = new SwerveModule("FrontLeft", Constants.Swerve.FL_POWER, Constants.Swerve.FL_STEER, Constants.Swerve.FL_ENCODER, driveTrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(0, 0));
    frontRight = new SwerveModule("FrontRight", Constants.Swerve.FR_POWER, Constants.Swerve.FR_STEER, Constants.Swerve.FR_ENCODER, driveTrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(2, 0));
    backLeft = new SwerveModule("BackLeft", Constants.Swerve.BL_POWER, Constants.Swerve.BL_STEER, Constants.Swerve.BL_ENCODER, driveTrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(4, 0));
    backRight = new SwerveModule("BackRight", Constants.Swerve.BR_POWER, Constants.Swerve.BR_STEER, Constants.Swerve.BR_ENCODER, driveTrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withPosition(6, 0));

    driveTrainTab.addDouble("Gyro Angle", () -> getRotation2d().getDegrees());        
    //driveTrainTab.add("field", field).withSize(8, 5).withPosition(1, 1);

    SmartDashboard.putData("Field", field);

    //took autobuilder from pathplanner - might need to be used in the auto file (driveRobotRelative not coded yet)
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative , // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(Constants.Auto.kPThetaController, 0.0, 0.0), // Rotation PID constants, TODO: check kpTheta
          0.3, // Max module speed, in m/s, TODO: check
          0.41309, // Drive base radius in meters. Distance from robot center to furthest module. 
          new ReplanningConfig() // Default path replanning config. See the API for the options here
    ), () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) return alliance.get() == DriverStation.Alliance.Red;
      else return false;
    }, this);

    resetGyro();
  }



  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }



  /** @return Rotation2d object with desired angle based on degrees from gyro */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(m_gyro.getAngle(m_gyro.getYawAxis()));
  }



  public double getYaw() {
    return m_gyro.getAngle(m_gyro.getYawAxis());
  }


  
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



  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
    this.setModuleStates(moduleState);
  }



  public static double signedSquare(double a) {
    if (a < 0) return -(a * a);
    else return a * a;
  }

  

  public double getEncoderPosition(){
    return frontRight.getEncoderPosition();
  }



  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setState(desiredStates[0]);
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }



  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }



  public void resetOdometry (Pose2d pose) {
    m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), getPose());
  }



  public void resetGyro () {
    m_gyro.setGyroAngle(m_gyro.getYawAxis(), 0);
  }



  public double getGyro () {
    return m_gyro.getAngle(m_gyro.getYawAxis());
  }



  public void defenseMode(){
    SwerveModuleState fLDefenseState= new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState fRDefenseState = new SwerveModuleState(0, Rotation2d.fromDegrees(135));
    SwerveModuleState bLDefenseState= new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState bRDefenseState = new SwerveModuleState(0, Rotation2d.fromDegrees(135));

    frontLeft.setState(fLDefenseState);
    frontRight.setState(fRDefenseState);
    backLeft.setState(bLDefenseState);
    backRight.setState(bRDefenseState);
    System.out.println("lessgooo");

    frontLeft.brakeMode();
    frontRight.brakeMode();
    backLeft.brakeMode();
    backRight.brakeMode();
  }



  public void setNeutralMode(boolean brake) {
    if (brake){
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



  public void setGyroAngle(double degrees){
    m_gyro.setGyroAngleZ(degrees);
  }


  public void configDashboard(ShuffleboardTab tab){
    tab.addDouble("Power Encoder Position", ()-> getEncoderPosition());
    tab.addDouble("pose position x", () -> getPose().getX());
    tab.addDouble("pose position y", () -> getPose().getY());
    tab.addDouble("Gyro Yaw Axis", () -> getGyro());
    tab.addBoolean("isReached?", () -> isInRange);
    //tab.addCamera("pose2d", "pose 2d", pose);

    tab.addDouble("Encoder Meters Moved", ()->getEncoderPosition() * Constants.Swerve.METERS_TO_ENC_COUNT);
    tab.addDouble("FR Distance Meters", ()-> frontRight.getPosition().distanceMeters);
    tab.addDouble("FL Distance Meters", ()-> frontLeft.getPosition().distanceMeters);
    tab.addDouble("BR Distance Meters", ()-> backRight.getPosition().distanceMeters);
    tab.addDouble("BL Distance Meters", ()-> backLeft.getPosition().distanceMeters);


    SmartDashboard.putNumber("Steer kS", 1);
    SmartDashboard.putNumber("Steer kV", 1);

    //tab.addDouble("steer reference", () -> frontLeft.getSteerReference().getValueAsDouble())
      //.withWidget(BuiltInWidgets.kGraph);
    //tab.addDouble("steer rot?", () -> frontLeft.getSteerPosition())
      //.withWidget(BuiltInWidgets.kGraph);
  }

  @Override
  public void periodic() {
    getEncoderPosition();
    frontLeft.periodic();
    frontRight.periodic();
    backLeft.periodic();
    backRight.periodic();

    var gyroAngle = getRotation2d();

    pose = m_poseEstimator.update(gyroAngle, new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
    });

    //System.out.println(pose);
    SmartDashboard.putData("Field", field);
    field.setRobotPose(pose);
  }
}