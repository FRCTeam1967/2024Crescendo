package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AmpReverseV2 extends Command {

    private final Swerve swerve;
    private DoubleSupplier ySupplier;
    private SlewRateLimiter yLimiter;
    private Timer timer;
    private double initialPos;
    private double finalPos;
    private boolean redAlliance;


   public AmpReverseV2(Swerve swerve, DoubleSupplier ySupplier, boolean redAlliance) {
        this.swerve = swerve;
        this.ySupplier = ySupplier;
        this.redAlliance = redAlliance;
        addRequirements(swerve);
    }

    public void initialize() {
        timer = new Timer();
        timer.start();
        initialPos = swerve.getEncoderPosition();
        finalPos = initialPos + 0.32495117;
        finalPos = initialPos + 0.16807819;
    }

    private double cleanAndScaleInput(double deadband, double input, SlewRateLimiter limiter, double speedScaling) {
        input = Math.pow(input, 3);
        input = Math.abs(input) > deadband ? input : 0;
        input *= speedScaling;

        return input;
    }

    
    @Override
    public void execute() {
        double ySpeed = cleanAndScaleInput(0.00, ySupplier.getAsDouble(), yLimiter, (Constants.Swerve.SWERVE_MAX_SPEED)/2);
        // if (redAlliance){
        //     ySpeed = -ySpeed;
        // }
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, 0, 0, swerve.getRotation2d());
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        //SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, Constants.Swerve.SWERVE_MAX_SPEED);
        swerve.setModuleStates(moduleState);
        
    }


    @Override
    public void end (boolean interrupted) {
        swerve.stopModules();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
      if ((timer.get() >= 1) || (swerve.getEncoderPosition() >= finalPos)){
        return true;
      }
        return false;
    }
}
