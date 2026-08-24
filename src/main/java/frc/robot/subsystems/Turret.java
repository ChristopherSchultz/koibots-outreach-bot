package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Fishstickks Turret.
 */
@Logged
public class Turret extends SubsystemBase {
    private static final double DIRECTLY_AHEAD_ANGLE = 0;
    private static final double MAX_PORT_ANGLE = -135;
    private static final double MAX_STARBOARD_ANGLE = 135;

    public Turret(boolean isRealRobot) {
    }

    private double targetAngle = DIRECTLY_AHEAD_ANGLE;

    protected void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    protected void stop() {
        // TODO
    }

    public Command getSetTargetAngleCommand(double targetAngle) {
        return Commands.runOnce(() -> setTargetAngle(targetAngle));
    }

    public Command getSweepToAngleCommand(double targetAngle) {
        return getSetTargetAngleCommand(targetAngle);
    }

    public Command getStopCommand() {
        return Commands.runOnce(() -> stop());
    }

    @Override
    public void periodic() {
        // TODO
    }

    public Command getOscillateCommand() {
        return Commands.repeatingSequence(
            getSweepToAngleCommand(MAX_PORT_ANGLE),
            getSweepToAngleCommand(MAX_STARBOARD_ANGLE)
        );
    }
}
