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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase{

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule backLeft;
    public final SwerveModule backRight;


    private final ADIS16470_IMU gyro;
    public final SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();

    private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
    private boolean isInRange = false;

    private Pose2d pose;

    public Swerve() {

        ShuffleboardTab driveTrainTab = Shuffleboard.getTab("Drivetrain");

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
        
        gyro = new ADIS16470_IMU();

        driveTrainTab.addDouble("Gyro Angle", () -> getRotation2d().getDegrees());        
        //driveTrainTab.add("field", field).withSize(8, 5).withPosition(1, 1);



        odometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, getRotation2d(), 
        new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });

        //took autobuilder from pathplanner - might need to be used in the auto file (driveRobotRelative not coded yet)
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative , // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(0.0001, 0.0, 0.0), // Translation PID constants
                new PIDConstants(Constants.Auto.kPThetaController, 0.0, 0.0), // Rotation PID constants
                0.3, // Max module speed, in m/s
                0.41309, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
        this);
        }
    
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //takes in degrees and returns rotation object with desired angle
    public Rotation2d getRotation2d() {
        //adis16470 version
        //var degrees = gyro.getAngle(gyro.getYawAxis());
        //return Rotation2d.fromDegrees(degrees);

        //pigeon version
        return Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis()));
    
    }

    public double getYaw() {
        //adis16470 version
        //return gyro.getAngle(gyro.getYawAxis());

        //pigeon version
        return gyro.getAngle(gyro.getYawAxis());
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

    //finish
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        this.setModuleStates(moduleState);
    }

    public static double signedSquare(double a) {
        if (a < 0) {
          return -(a * a);
        }
        return a * a;
    }

    public boolean isInRange(double goalRevs, double currentRevs){
        double offsetRevs = (goalRevs-currentRevs);
        if (offsetRevs > -Constants.Swerve.REVERSE_OFFSET_REVS && offsetRevs < Constants.Swerve.REVERSE_OFFSET_REVS){
            isInRange = true;
        } else {
            isInRange = false;
        }
        return isInRange;
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
        return pose;
    }

    public void resetOdometry (Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void resetGyro () {
        gyro.setGyroAngle(gyro.getYawAxis(), 0);
    }

    public double getGyro () {
        return gyro.getAngle(gyro.getYawAxis());
    }

    public void goToAngle(double angleInDegrees){
        SwerveModuleState fixedDegree = new SwerveModuleState(0, Rotation2d.fromDegrees(angleInDegrees));
        frontLeft.setState(fixedDegree);
        frontRight.setState(fixedDegree);
        backLeft.setState(fixedDegree);
        backRight.setState(fixedDegree);
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

    public void configDashboard(ShuffleboardTab tab){
        tab.addDouble("Power Encoder Position", ()-> getEncoderPosition());
        tab.addDouble("pose position x", () -> getPose().getX());
        tab.addDouble("pose position y", () -> getPose().getY());
        tab.addDouble("Gyro Yaw Axis", () -> getGyro());
        tab.addBoolean("isReached?", () -> isInRange);
    }


    @Override
    public void periodic() {
        getEncoderPosition();
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        var gyroAngle = getRotation2d();

        pose = odometry.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
          });

          //System.out.println(pose);
          field.setRobotPose(pose);
    }

}
