// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */



public class PitCommands {

    public static Command zeroSwerve(Swerve swerve) {
        return new InstantCommand(()->{swerve.zeroizeSteer();});
    }


}
