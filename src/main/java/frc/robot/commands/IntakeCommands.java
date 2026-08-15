package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtension;

public class IntakeCommands {
    public static Command getRunIntakeCommand(IntakeExtension intakeExtension,
                                              Intake intake)
    {
        return intakeExtension.deployExtension()
               .until(intakeExtension::getForwardLimit)
               .andThen(intake.runIntake());
    }

    public static Command getStopIntakeCommand(IntakeExtension intakeExtension,
                                               Intake intake)
    {
        return intakeExtension.retractExtension()
               .until(intakeExtension::getReverseLimit)
               .andThen(intake.idleIntake());
    };
}