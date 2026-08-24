package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents the Fishsticks shooter. This is only the actual shooter part of the robot, and not the
 * turret, which can be found elsewhere.
 *
 * @see Turret
 */
@Logged
public class Shooter extends SimpleSpinningSubsystem {
    /**
     * The target speed of the shooter.
     */
    public static final AngularVelocity TARGET_SPEED = RPM.of(2000);

    /**
     * The difference between the target velocity and the actual velocity we will tolerate.
     */
    public static final AngularVelocity VELOCITY_EPSILON = RPM.of(100);

    public static final AngularAcceleration MAX_ACCELERATION = RPM.per(Second).of(2000);

    public static final int MOTOR_ID = 50;

    public Shooter(boolean isRealRobot) {
        super(isRealRobot, MOTOR_ID, DEFAULT_CURRENT_LIMIT, NEO_MAX, MAX_ACCELERATION, VELOCITY_EPSILON);
    }

    public Command startShooting() {
        return setTargetVelocityCommand(TARGET_SPEED);
    }

    public Command stopShooting() {
        return stopCommand();
    }
}
