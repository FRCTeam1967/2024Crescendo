package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;


public enum AutonomousRoutines {
  DEFAULT_AUTO(false, "DEFAULT", Commands.print("DEFAULT AUTO SAYS HI")),
  SQUARE_AUTO(true, "Square", "Square");

  public final boolean showInDashboard;
  public final String shuffleboardName;
  public final String trajectoryName;
  public final boolean buildable;

//   public pathplanner

  public Command command;
  public Command builtCommand;

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

  public void build(Swerve drivetrain) {
    if (buildable) {
      // Add path planner initialization
      // builtCommand = PathPlanner.()
      // This is also the place to add other configuration and "set up" or "clean up" options
    } else {
      builtCommand = command;
    }
  }
}