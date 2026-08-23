package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class ShooterCommands {
    public static Command getStartShootingCommand(Spindexer spindexer, Feeder feeder, Shooter shooter) {
        return shooter.startShooting()
                .andThen(Commands.waitUntil(() -> shooter.isAtTargetVelocity())
                .andThen(feeder.startFeeding())
                .andThen(Commands.waitUntil(() -> feeder.isAtTargetVelocity()))
                .andThen(spindexer.startSpinning()));
    }

    public static Command getStopShootingCommand(Spindexer spindexer, Feeder feeder, Shooter shooter) {
        return spindexer.stopSpinning()
        .andThen(feeder.stopFeeding())
        .andThen(shooter.stopShooting())
        ;
    };
}
