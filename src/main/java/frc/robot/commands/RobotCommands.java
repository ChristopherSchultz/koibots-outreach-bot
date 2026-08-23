package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Eyes;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class RobotCommands {
    public static Command getKillAllHumansCommand(Subsystem drivetrain, Spindexer spindexer, Feeder feeder, Subsystem turret, Shooter shooter, Eyes eyes) {
        return // drivetrain.stalk().andThen
            //.andThen(
            eyes.getAngryCommand()
            .andThen(ShooterCommands.getStartShootingCommand(spindexer, feeder, shooter))
            // .andThen(feeder.getOscillateCommand())
            ;
    }

    public static Command getStealthModeCommand(Subsystem drivetrain, Spindexer spindexer, Feeder feeder,
            Subsystem turret, Shooter shooter, Eyes eyes) {
                return
                // driveTrain.stop() // active stop
                // .andThen(turret.stop()) // active stop?
                ShooterCommands.getStopShootingCommand(spindexer, feeder, shooter)
                .andThen(eyes.getActCasualCommand())
                ;
    }
}