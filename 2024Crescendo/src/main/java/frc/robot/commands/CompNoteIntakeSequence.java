package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class CompNoteIntakeSequence {
    public static Command start(String name, Pivot pivot, Intake intake, Feeder feeder) {
        var cmd = new SequentialCommandGroup(
                new RunPivotIntakeBeam(pivot, intake, feeder),
                new ReverseBeamFeeder(feeder));

        cmd.setName(name);
        SmartDashboard.putData(cmd);
        return cmd;
    }

    public static Command startWithRumble(String name, Pivot pivot, Intake intake, Feeder feeder,
            CommandXboxController driver, CommandXboxController operator) {
        var cmd = new SequentialCommandGroup(
                new RunPivotIntakeBeam(pivot, intake, feeder),
                new ReverseBeamFeeder(feeder),
                new RumbleController(driver, operator).withTimeout(2));

        cmd.setName(name);
        SmartDashboard.putData(cmd);
        return cmd;
    }

    public static Command end(String name, Pivot pivot) {
        var endCmd = new MovePivot(pivot, Constants.Pivot.INTAKE_SAFE);
        endCmd.setName(name);
        SmartDashboard.putData(endCmd);
        return endCmd;
    }

}
