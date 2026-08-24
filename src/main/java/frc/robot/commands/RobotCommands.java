package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class RobotCommands {
    public static Command getKillAllHumansCommand(Subsystem drivetrain, Spindexer spindexer, Feeder feeder,
            Turret turret, Shooter shooter, Eyes eyes) {
        return // drivetrain.stalk()
        Commands.none() // Just until we have a drivetrain
                .andThen(eyes.getAngryCommand())
                .andThen(ShooterCommands.getStartShootingCommand(spindexer, feeder, shooter))
                .andThen(turret.getOscillateCommand());
    }

    public static Command getStealthModeCommand(Subsystem drivetrain, Spindexer spindexer, Feeder feeder, Turret turret,
            Shooter shooter, Eyes eyes) {
        return Commands.parallel(// drivetrain.stop(),
                turret.getStopCommand(), // active stop?
                ShooterCommands.getStopShootingCommand(spindexer, feeder, shooter), eyes.getActCasualCommand());
    }
}
