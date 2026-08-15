package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    public static Command getStartShootingCommand(Shooter shooter) {
        return shooter.startShooting();
    }

    public static Command getStopShootingCommand(Shooter shooter) {
        return shooter.stopShooting();
    };
}
