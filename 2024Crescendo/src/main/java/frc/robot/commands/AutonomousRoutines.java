package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.*;

import java.util.Vector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public enum AutonomousRoutines {
  DEFAULT_AUTO(false, "DEFAULT", Commands.print("DEFAULT AUTO SAYS HI")),
  SQUARE_AUTO(true, "Square", "Square");

  public final boolean showInDashboard;
  public final String shuffleboardName, trajectoryName;
  public final boolean buildable;

  private PathLegs[] pathOne; //scorePreloadIntakeMiddle + intakemiddle

//   public pathplanner

  public Command command, builtCommand;

  AutonomousRoutines(
    boolean show, String shuffleboardName, String trajectoryName
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    this.trajectoryName = trajectoryName;
    buildable = true;
  }

  AutonomousRoutines(String shuffleboardName, String trajectoryName) {
    this(true, shuffleboardName, trajectoryName);
  }

  AutonomousRoutines(
    boolean show, String shuffleboardName, Command simpleCommand
  ) {
    showInDashboard = show;
    this.shuffleboardName = shuffleboardName;
    command = simpleCommand;
    buildable = false;

    trajectoryName = null;
  }

  public void build(Swerve swerve) {
    if (buildable) {
      Vector<PathPlannerTrajectory> pathSet = new Vector<PathPlannerTrajectory>();
      for (PathLegs p : pathOne){
        if (p.currentlyExists){
          //pathSet.add(PathPlannerPath.registerHotReloadPath​(p.name, )); //Hot reload: paths and autos can be updated and regenerated on the robot without redeploying code
        }
      }
      // Add path planner initialization
      // builtCommand = PathPlanner.()
      // This is also the place to add other configuration and "set up" or "clean up" options
    } else {
      builtCommand = command;
    }
  }

  public enum PathLegs {
    IntakeMiddle(true, "IntakeMiddle"), //path
    ShootMiddle(true, "ShootMiddle"), //path
    
    scorePreloadIntakeMiddle(true, "scorePreloadIntake"), //auto
    scorePreloadScoreMiddle(true, "scorePreloadScoreMiddle"); //auto
    
    public final boolean currentlyExists;
    public final String name;

    PathLegs (boolean currentlyExists, String name) {
      this.currentlyExists = currentlyExists;
      this.name = name;
    }

    public boolean currentlyExists (boolean currentlyExists){
      return currentlyExists;
    }

    public String name(String name){
      return name;
    }
  }
}