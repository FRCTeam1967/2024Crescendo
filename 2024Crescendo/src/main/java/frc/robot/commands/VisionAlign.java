package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class VisionAlign extends Command {
    private double lastPosition;
    private final Swerve swerve;
    private final Vision vision;

    private final ProfiledPIDController translateController =
        new ProfiledPIDController(3, 0, 0.0, Constants.Swerve.SWERVE_TRANSLATION_PID_CONSTRAINTS);

    public VisionAlign(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

        translateController.setTolerance(0.0001);
        translateController.enableContinuousInput(0, 1);
    }

    public void initialize() {
        translateController.reset(swerve.getPose().getY(), 0);
        lastPosition = swerve.getPose().getY();
    }

//     private double cleanAndScaleInput(double deadband, double input, double speedScaling) {
//       input = Math.pow(input, 3);
//       input = Math.abs(input) > deadband ? input : 0;
//       input *= speedScaling;

//       return input;
//   }

    public void execute() {
       // double translateSpeed = cleanAndScaleInput(0.00, translateController.calculate(swerve.getPose().getY(), -vision.getOffset()), 1);
        double translateSpeed = translateController.calculate(swerve.getPose().getY(), lastPosition+(vision.getOffset()*0.0254));

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, translateSpeed, 0, swerve.getRotation2d());
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, Constants.Swerve.SWERVE_MAX_SPEED);
        swerve.setModuleStates(moduleState);
    }

    public void end (boolean interrupted) {
        swerve.stopModules();
    }

    public boolean isFinished() {
      return vision.getIsInRange();
    }
}
