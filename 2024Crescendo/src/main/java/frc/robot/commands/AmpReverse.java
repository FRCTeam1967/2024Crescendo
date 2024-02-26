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

public class AmpReverse extends Command {
    private double lastPosition, goalPosition;
    private double initialPositionRevs, goalPositionRevs, currentPositionRevs;
    private final Swerve swerve;
    private boolean redAlliance;

    private final ProfiledPIDController translateController =
            new ProfiledPIDController(0.8, 0, 0.0, Constants.Swerve.SWERVE_TRANSLATION_PID_CONSTRAINTS);

    public AmpReverse(Swerve swerve, boolean redAlliance) {
        this.swerve = swerve;
        this.redAlliance = redAlliance;
        translateController.setTolerance(0.0001);
        addRequirements(swerve);
    }

    public void initialize() {
        //initialPositionRevs = swerve.getEncoderPosition();
        //goalPositionRevs = initialPositionRevs + ((3.75/(4*Math.PI)) * Constants.Swerve.DRIVE_GEAR_RATIO);
        //goalPositionRevs = initialPositionRevs + Constants.Swerve.SWERVE_AMP_OFFSET;
        //currentPositionRevs = swerve.getEncoderPosition();

        //translateController.reset(currentPositionRevs, 0);
        lastPosition = swerve.getPose().getY();
        goalPosition = lastPosition + 0.09525;
        System.out.println("\n\n\n goalPosition: " + goalPosition); 
        System.out.println("\n\n\n goalPositionRevs: " + goalPositionRevs); 
    }

  //   private double cleanAndScaleInput(double deadband, double input, double speedScaling) {
  //     input = Math.pow(input, 3);
  //     input = Math.abs(input) > deadband ? input : 0;
  //     input *= speedScaling;

  //     return input;
  // }


    public void execute() {
        currentPositionRevs = swerve.getEncoderPosition();
        double translateSpeed = translateController.calculate(swerve.getPose().getY(), goalPosition);
        //double translateSpeed = translateController.calculate(currentPositionRevs, goalPositionRevs);
        // if (redAlliance){
        //     translateSpeed = -translateSpeed;
        // }
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, Math.abs(translateSpeed), 0, swerve.getRotation2d());
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, Constants.Swerve.SWERVE_MAX_SPEED);
        swerve.setModuleStates(moduleState);
    }



    public void end (boolean interrupted) {
        swerve.stopModules();
    }

    public boolean isFinished() {
      return swerve.isInRange(goalPositionRevs, currentPositionRevs);
    }


}
