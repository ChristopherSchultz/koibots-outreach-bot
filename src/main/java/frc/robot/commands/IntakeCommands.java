package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtension;

public class IntakeCommands {
    public static Command getRunIntakeCommand(IntakeExtension intakeExtension, Intake intake) {
        // NOTE: The forward-limit will automatically stop the motor,
        // so there is no reason to specifically program a stop-condition
        // into this command.
        return intakeExtension.deployExtension().until(intakeExtension::getForwardLimit).andThen(intake.runIntake());
    }

    public static Command getStopIntakeCommand(IntakeExtension intakeExtension, Intake intake) {
        // NOTE: The forward-limit will automatically stop the motor,
        // so there is no reason to specifically program a stop-condition
        // into this command.
        return intakeExtension.retractExtension().until(intakeExtension::getReverseLimit).andThen(intake.idleIntake());
    };

    public static Command getReverseIntakeCommand(Intake intake) {
        return intake.reverse();
    }
}