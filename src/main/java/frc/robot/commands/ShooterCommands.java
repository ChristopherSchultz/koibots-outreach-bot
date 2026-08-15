package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class ShooterCommands {
    public static Command getStartShootingCommand(Spindexer spindexer, Shooter shooter) {
        return shooter.startShooting().andThen(spindexer.startSpinning());
    }

    public static Command getStopShootingCommand(Spindexer spindexer, Shooter shooter) {
        return spindexer.stopSpinning().andThen(shooter.stopShooting());
    };
}
