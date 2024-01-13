package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.Constants;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase{

    public final SwerveModule frontLeft;
    public final SwerveModule frontRight;
    public final SwerveModule backLeft;
    public final SwerveModule backRight;

    private final Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;

    private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

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
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Canivore");

        driveTrainTab.addDouble("Falcon Gyro Angle", () -> gyro.getAngle());
        
        odometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, getRotation2d(), 
        new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });

        //took autobuilder from pathplanner - might need to be used in the auto file (driveRobotRelative not coded yet)
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ), 
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
        return Rotation2d.fromDegrees(gyro.getAngle());
    
    }

    public double getYaw() {
        //adis16470 version
        //return gyro.getAngle(gyro.getYawAxis());

        //pigeon version
        return gyro.getAngle();
    }
    
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(), 
            backLeft.getState(),
            backRight.getState()
        };
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed){
        xLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED * 2);
        yLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED * 2);
        rotationLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);

        //speed is from xsupplier  value and scaled down by max speed
        double xSpeedScaled = cleanAndScaleInput(xSpeed, xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double ySpeedScaled = cleanAndScaleInput(ySpeed, yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        double rotationSpeedScaled = cleanAndScaleInput(rotationSpeed, rotationLimiter, Constants.Swerve.SWERVE_ROTATION_MAX_SPEED);
        //converts field relative speeds to robot relative speeds 
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedScaled, ySpeedScaled, rotationSpeedScaled, this.getRotation2d());
        //converts new chassisspeeds to module states
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        //ensures wheel speeds do not exceed swerve max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, Constants.Swerve.SWERVE_MAX_SPEED);
        this.setModuleStates(moduleState);
    }

    //finish
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond * Constants.Auto.RADIANS_TO_DEGREES);
    }

    private double cleanAndScaleInput(double input, SlewRateLimiter limiter, double speedScaling) {
        //lowering down to 0 if abs value of input is below deadband 
        input = Math.abs(input) > Constants.Swerve.SWERVE_DEADBAND ? input : 0;
        //squaring input while preserving sign
        input = signedSquare(input);
        //using slewratelimiter to scale input
        input = limiter.calculate(input);
        input *= speedScaling;

        return input;
    }

    public static double signedSquare(double a) {
        if (a < 0) {
          return -(a * a);
        }
        return a * a;
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
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        }, pose);
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

}
