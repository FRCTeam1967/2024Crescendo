// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.janksters.ExampleCommonClasses.Commands.DrawRectangleCommand;
import org.janksters.ExampleCommonClasses.Commands.ScrollingTextCommand;
import org.janksters.ExampleCommonClasses.Commands.TextCommand;
import org.janksters.ExampleCommonClasses.Commands.TextShiftingCommand;
import org.janksters.ExampleCommonClasses.Commands.ValueDisplayCommand;
import org.janksters.ExampleCommonClasses.Drawing.BitmapFont;
import org.janksters.ExampleCommonClasses.Drawing.BitmapFontManager;
import org.janksters.ExampleCommonClasses.Drawing.Point;
import org.janksters.ExampleCommonClasses.Drawing.Rectangle;
import org.janksters.ExampleCommonClasses.Subsystems.LEDPanelSubsystem;
import org.janksters.ExampleCommonClasses.Subsystems.LEDRegionSubsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LEDPanelSubsystem m_ledSubsystem;
  private final LEDRegionSubsystem m_upperHalf;
  private final LEDRegionSubsystem m_lowerHalf;
  private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private ShuffleboardTab limelightTab = Shuffleboard.getTab("limelight tab");
  private Vision vision = new Vision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandJoystick m_joystickController =
  //     new CommandJoystick(OperatorConstants.kDriverControllerPort);

  private BitmapFontManager m_fontManager = new BitmapFontManager();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    vision.configDashboard(limelightTab);
    m_ledSubsystem = new LEDPanelSubsystem(Constants.LEDConstants.kMatrixWidth, Constants.LEDConstants.kMatrixHeight, Constants.LEDConstants.kLEDPWMPin);
    m_ledSubsystem.brightness = 0.10;

    m_upperHalf = new LEDRegionSubsystem(m_ledSubsystem, Constants.LEDConstants.LEDRegions.kUpperHalf);
    m_lowerHalf = new LEDRegionSubsystem(m_ledSubsystem, Constants.LEDConstants.LEDRegions.kLowerHalf);
    
    // Configure the Encoder



    // m_ledSubsystem.setDefaultCommand(new RainbowCommand(m_ledSubsystem).ignoringDisable(true));

    File deployDirectory = Filesystem.getDeployDirectory();
    m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kSmallFontFile)));
    // m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kMediumFontFile)));
    m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kLargeFontFile)));
    // m_fontManager.addFont(new BitmapFont(new File(deployDirectory, Constants.Fonts.kMediumAltFontFile)));

    // Configure the trigger bindings
    configureBindings();
    
    // // Note: Subsystem default commands must require their subsystem. So at least one of the commands in a 
    // // group assigned to m_ledSubsystem must require it.
    // if (true) {
    //   m_ledSubsystem.setDefaultCommand(fontTestCommand());
    // } else {
    //   var smallFont = m_fontManager.getFont(Constants.Fonts.kSmallFontFile);
    //   m_upperHalf.setDefaultCommand(
    //     new TextShiftingCommand("Janksters!", () -> m_encoder.getAbsolutePosition().getValueAsDouble(), new Point(0, 0), smallFont, Color.kRed, m_upperHalf)
    //       .ignoringDisable(true)
    //   );
    //   m_lowerHalf.setDefaultCommand(
    //     new ValueDisplayCommand(() -> m_encoder.getAbsolutePosition().getValueAsDouble(), new Point(0, 0), smallFont, Color.kBlue, m_lowerHalf)
    //       .ignoringDisable(true)
    //   );
    // }
  }

  public Command alignLED() {
      var smallFont = m_fontManager.getFont(Constants.Fonts.kSmallFontFile);
      
      Command commandGroup;
      if (vision.alignAngle()){
        commandGroup = new SequentialCommandGroup (
          new InstantCommand(() -> m_ledSubsystem.clearScreen(Color.kBlack), m_ledSubsystem, m_lowerHalf, m_upperHalf),
          new ParallelCommandGroup(
          new TextCommand("Janksters", new Point(0, 0), smallFont, Color.kRed, m_upperHalf),
          new TextCommand("Aligned", new Point(0, 0), smallFont, Color.kGreen, m_lowerHalf)), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
      } 
      else if (vision.getOffset() < -Constants.Vision.DEGREE_ERROR){
        commandGroup = new SequentialCommandGroup (
          new InstantCommand(() -> m_ledSubsystem.clearScreen(Color.kBlack), m_ledSubsystem, m_lowerHalf, m_upperHalf),
          new ParallelCommandGroup(
          new TextCommand("Janksters", new Point(0, 0), smallFont, Color.kRed, m_upperHalf),
          new TextCommand("Move Left", new Point(0, 0), smallFont, Color.kRed, m_lowerHalf)), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
       
      } else if (vision.getOffset() > Constants.Vision.DEGREE_ERROR){
        commandGroup = new SequentialCommandGroup (
          new InstantCommand(() -> m_ledSubsystem.clearScreen(Color.kBlack), m_ledSubsystem, m_lowerHalf, m_upperHalf),
          new ParallelCommandGroup(
          new TextCommand("Janksters", new Point(0, 0), smallFont, Color.kRed, m_upperHalf),
          new TextCommand("Move Right", new Point(0, 0), smallFont, Color.kRed, m_lowerHalf)), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
        
      } else{
        commandGroup = new SequentialCommandGroup (
          new InstantCommand(() -> m_ledSubsystem.clearScreen(Color.kBlack), m_ledSubsystem, m_lowerHalf, m_upperHalf),
          new ParallelCommandGroup(
          new TextCommand("Janksters", new Point(0, 0), smallFont, Color.kRed, m_upperHalf),
          new TextCommand("Not Seen", new Point(0, 0), smallFont, Color.kDarkRed, m_lowerHalf)), 
          new WaitCommand (1.0)
        ).ignoringDisable(true);
      }
    return commandGroup;
  }
 
  //takes command from method, schedules command
      public void LEDscheduler (){
         CommandScheduler.getInstance().schedule(alignLED());
      }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_joystickController.button(1).whileTrue(new TextCommand("Hello World!", m_ledSubsystem));

    
    //vision.setDefaultCommand(new InstantCommand (() -> LEDscheduler()).ignoringDisable(true));
    m_controller.y().onTrue(new InstantCommand (() -> LEDscheduler()).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return alignLED();
  }

 
}



