package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Eyes extends SubsystemBase {
    private final Eye leftEye, rightEye;

    public Eyes(boolean isRealRobot) {
        leftEye = new Eye(isRealRobot);
        rightEye = new Eye(isRealRobot);
    }

    public Command getNeutralCommand() {
        return Commands.parallel(leftEye.getLookStraightCommand(),
                                 rightEye.getLookStraightCommand()
        );
    }

    public Command getAngryCommand() {
        return Commands.parallel(leftEye.getLookDownRightCommand(),
                                 rightEye.getLookDownLeftCommand())
        ;
    }

    public Command getSadCommand() {
        return Commands.parallel(leftEye.getLookDownCommand(),
                                 rightEye.getLookDownCommand())
        ;
    }

    public Command getCrossEyedCommand() {
        return Commands.parallel(leftEye.getLookRightCommand(),
                                 rightEye.getLookLeftCommand())
        ;
    }

    public Command getActCasualCommand() {
        return getNeutralCommand()
        .andThen(Commands.waitUntil(leftEye::isAtTargetAngle))
        .andThen(Commands.waitUntil(rightEye::isAtTargetAngle))
        .andThen(Commands.parallel(leftEye.getOscillateBetweenCommand(Eye.ANGLE_NORTHWEST, Eye.ANGLE_NORTHEAST)))
        ;
    }
}
