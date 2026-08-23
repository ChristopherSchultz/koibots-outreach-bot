package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents the Fishsticks intake. This is only the actual intake part of the robot, and not the
 * extension, which can be found elsewhere.
 *
 * @see IntakeExtension
 */
@Logged
public class Intake extends SimpleSpinningSubsystem {
    public static final AngularVelocity TARGET_SPEED = RPM.of(2000);

    public static final AngularVelocity REVERSE_SPEED = RPM.of(-50);

    /**
     * The difference between the target velocity and the actual velocity we will tolerate.
     */
    public static final AngularVelocity VELOCITY_EPSILON = RPM.of(100);

    public static final int MOTOR_ID = 10;

    public Intake(boolean isRealRobot) {
        super(isRealRobot, MOTOR_ID, DEFAULT_CURRENT_LIMIT, NEO_MAX, DEFAULT_MAX_ACCELERATION, VELOCITY_EPSILON);
    }

    public Command runIntake() {
        return setTargetVelocityCommand(TARGET_SPEED);
    }

    public Command idleIntake() {
        return stopCommand();
    }

    public Command reverseIntake() {
        return setTargetVelocityCommand(REVERSE_SPEED);
    }
}
