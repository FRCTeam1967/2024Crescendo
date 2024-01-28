package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends Command {

    private final Swerve swerve;
    private DoubleSupplier xSupplier, ySupplier, rotationSupplier;
    private SlewRateLimiter xLimiter, yLimiter, rotationLimiter;

   public SwerveDrive (Swerve swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        this.swerve = swerve;
        //suppliers will grab double values (needed for grabbing joystick values for driving)
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        //ramp up speed at a rate limit of swerve max * 2 per second
        //xLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED * 400);
        //yLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_MAX_SPEED * 400);
        //rotationLimiter = new SlewRateLimiter(Constants.Swerve.SWERVE_ROTATION_MAX_SPEED * 400);
        //command uses the swerve subsystem
        addRequirements(swerve);
    }

    private double cleanAndScaleInput(double input, SlewRateLimiter limiter, double speedScaling) {
        //lowering down to 0 if abs value of input is below deadband 
        input = Math.abs(input) > Constants.Swerve.SWERVE_DEADBAND ? input : 0;
        //squaring input while preserving sign
        input = signedSquare(input);
        //using slewratelimiter to scale input
        //input = limiter.calculate(input);
        input *= speedScaling;

        return input;
    }
    
    @Override
    //main body of command, called repeatedly while command is scheduled
    public void execute() {
        //speed is from xsupplier joystick value and scaled down by max speed
        double xSpeed = cleanAndScaleInput(xSupplier.getAsDouble(), xLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        // System.out.println("XSPEED before CAS: " + xSupplier.getAsDouble());
        // System.out.println("XSPEED after CAS: " + xSpeed);
        double ySpeed = cleanAndScaleInput(ySupplier.getAsDouble(), yLimiter, Constants.Swerve.SWERVE_MAX_SPEED);
        // System.out.println("YSPEED before CAS: " + ySupplier.getAsDouble());
        // System.out.println("YSPEED after CAS: " + ySpeed);
        double rotationSpeed = cleanAndScaleInput(rotationSupplier.getAsDouble(), rotationLimiter, Constants.Swerve.SWERVE_ROTATION_MAX_SPEED_IN_RAD);
        // System.out.println("R_SPEED before CAS: " + rotationSupplier.getAsDouble());
        // System.out.println("R_SPEED after CAS: " + rotationSpeed);
        //converts field relative speeds to robot relative speeds 
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, swerve.getRotation2d());
        //System.out.println("chassis speeds: " + chassisSpeeds.toString());
        //converts new chassisspeeds to module states
        SwerveModuleState[] moduleState = Constants.Swerve.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        //System.out.println("module state: " + moduleState[0].toString());
        //ensures wheel speeds do not exceed swerve max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, Constants.Swerve.SWERVE_MAX_SPEED);
        //System.out.println(moduleState[0].toString());
        swerve.setModuleStates(moduleState);
        
    }


    @Override
    public void end (boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    //once finished, scheduler will end and un-schedule the method, right now we don't want it to un-schedule
    public boolean isFinished() {
        return false;
    }

    public static double signedSquare(double a) {
        if (a < 0) {
          return -(a * a);
        }
        return a * a;
    }
    

}