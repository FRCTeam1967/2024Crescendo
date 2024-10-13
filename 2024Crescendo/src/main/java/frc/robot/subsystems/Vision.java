package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    private final Field2d m_field = new Field2d();
    private final Field2d m_field2 = new Field2d();

    ShuffleboardTab visionData = Shuffleboard.getTab("VISION");

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("PersianPoseOdometryVision", Pose2d.struct).publish();

    private StructPublisher<Pose2d> posePublisher2 = NetworkTableInstance.getDefault()
            .getStructTopic("PersianPoseOdometryOnly", Pose2d.struct).publish();

    private Swerve swerve;

    @SuppressWarnings("unused")

    public final SwerveDrivePoseEstimator poseEstimator;

    public final SwerveDriveOdometry poseOdometryEstimator;

    public Vision(Swerve swerve) {

        this.swerve = swerve;

        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, new Rotation2d(), swerve.getModulePositions(), new Pose2d());
        
        this.poseOdometryEstimator = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, new Rotation2d(), swerve.getModulePositions(), new Pose2d());

        initShuffleboard();
    }

    public double getTheta() {
        return getCurrentPose().getRotation().getRadians();
    }

    public void updateOdometry() {
        poseEstimator.update(swerve.getRotation2d().times(-1), swerve.getModulePositions());
        poseOdometryEstimator.update(swerve.getRotation2d().times(-1), swerve.getModulePositions()); // testing
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // testing
    public Pose2d getCurrentOdoPose() {
        return poseOdometryEstimator.getPoseMeters();
    }

    // testing
    public void resetOdometry(Pose2d pose) {
        poseOdometryEstimator.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), pose);
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(swerve.getRotation2d(), swerve.getModulePositions(), pose);

        resetOdometry(pose);
    }

    public boolean tagDetected() {
        return LimelightHelpers.getTV("limelight");
    }

    public int getNumTags() {
        return getVisionPose().tagCount;
    }

    public double getAvgTagDist() {
        return getVisionPose().avgTagDist;
    }

    public LimelightTarget_Fiducial[] getTagFiducial() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Fiducials;
    }

    public double[] getTagIDs() {
        var fiducials = getTagFiducial();
        double[] ids = new double[fiducials.length];
        for (int i = 0; i < ids.length; i++) {
            ids[i] = fiducials[i].fiducialID;
        }
        return ids;
    }

    public boolean isIdDetected(double targetID) {
        for (double id : getTagIDs()) {
            if (targetID == id)
                return true;
        }
        return false;
    }

    public double getDistanceTo(Translation2d location) {
        return getCurrentPose().getTranslation().getDistance(location);
    }

    /**
     * Finds the distance from the robot to a certain location on the field. Uses
     * the robot odometry (no vision) to determine distance.
     * 
     * @param location The location to check the distance to.
     * @return The distance to the location.
     */
    public double getOdoDistanceTo(Translation2d location) {
        return getCurrentOdoPose().getTranslation().getDistance(location);
    }

    // public double getTX() {
    //   return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    // }

    // VISION UPDATING - VERSIONS 1-4

    // LimelightLib-given constants for Std Devs
    // public void updateVision1() {
    // PoseEstimate poseEstimate = getVisionPose();

    // if (poseEstimate.tagCount >= 2) {

    // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7,
    // 999999999)); // documentation defaults,
    // // change?
    // poseEstimator.addVisionMeasurement(poseEstimate.pose,
    // poseEstimate.timestampSeconds);
    // }

    // }

    // distant dependant formula for Std Devs
    public void updateVision2() {
        PoseEstimate poseEstimate = getCurrentPose();

        if (tagDetected()) {
            poseEstimator.setVisionMeasurementStdDevs(getEstimationStdDevs(poseEstimate));
            poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs(PoseEstimate estimatedPose) {
        int numTags = estimatedPose.tagCount;
        double avgDist = estimatedPose.avgTagDist;

        // var estStdDevs = VecBuilder.fill(4, 4, 8); // kSingleTagDevs
        var estStdDevs = VecBuilder.fill(5, 5, 9);

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            // estStdDevs = VecBuilder.fill(0.5, 0.5, 1); // kMultiTagDevs
            estStdDevs = VecBuilder.fill(1, 1, 2);
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        // Increase std devs based on (average) distance
        else {
            // estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            estStdDevs = estStdDevs.times((1 + (avgDist * avgDist / 30)) * 0.80);
        }
        return estStdDevs;
    }

    public void updateVision5() {
        LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        // invalid LL data
        if (botPose.pose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
                .getDistance(botPose.pose.getTranslation());
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization#using-wpilibs-pose-estimator
        if (getNumTags() > 0) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (getNumTags() >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (botPose.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (botPose.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            poseEstimator.setVisionMeasurementStdDevs(
                    VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            poseEstimator.addVisionMeasurement(botPose.pose,
                    Timer.getFPGATimestamp() - botPose.latency);
        }
    }

    // SHUFFLEBOARD

    public void updateField() {
        m_field.setRobotPose(getCurrentPose());
        m_field2.setRobotPose(getCurrentOdoPose());
    }

    public void snapOdometry() {
        resetOdometry(getCurrentPose());
    }

    public void snapVisionOdo() {
        resetOdometry(getCurrentOdoPose());
    }

    public double getDifference() {
        return getCurrentPose().getTranslation().getDistance(getCurrentPose().getTranslation());
    }

    public void updateNetworkTable() {
        posePublisher.set(poseEstimator.getEstimatedPosition());
        posePublisher2.set(poseOdometryEstimator.getPoseMeters());
    }

    public void configDashboard(ShuffleboardTab visionData) {
        visionData.add("Odometry + Vision Field", m_field).withWidget(BuiltInWidgets.kField);
        visionData.add("Odometry Only Field", m_field2).withWidget(BuiltInWidgets.kField);
        visionData.addNumber("Num Tags", () -> getNumTags());
        visionData.addNumber("Tag Dist", () -> getAvgTagDist());
        visionData.addNumber("Difference btw Current Pose and New Vision Estimate", () -> getDifference());
        visionData.add("Snap Odometry to Vision+Odometry", new InstantCommand(() -> snapOdometry()));
        visionData.add("Snap Vision+Odometry to Odometry", new InstantCommand(() -> snapVisionOdo()));
    }

}