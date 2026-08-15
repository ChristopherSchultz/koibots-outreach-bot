package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents the Fishsticks spindexer. This is only the actual spindexer part of the robot, and not
 * the shooter, which can be found elsewhere.
 *
 * @see Shooter
 */
@Logged
public class Spindexer extends SimpleSpinningSubsystem {
    public static final AngularVelocity TARGET_SPEED = RPM.of(2000);

    /**
     * The difference between the target velocity and the actual velocity we will tolerate.
     */
    public static final AngularVelocity VELOCITY_EPSILON = RPM.of(100);

    public static final int MOTOR_ID = 30;

    public Spindexer(boolean isRealRobot) {
        super(isRealRobot, MOTOR_ID, DEFAULT_CURRENT_LIMIT, NEO_MAX, DEFAULT_MAX_ACCELERATION, VELOCITY_EPSILON);
    }

    public Command startSpinning() {
        return setTargetVelocityCommand(TARGET_SPEED);
    }

    public Command stopSpinning() {
        return setTargetVelocityCommand(ZERO_VELOCITY);
    }
}
