package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
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
    public static final AngularVelocity TARGET_SPEED = Units.RPM.of(2000);

    public static final int MOTOR_ID = 0;

    public Intake(boolean isRealRobot) {
        super(isRealRobot, MOTOR_ID, DEFAULT_CURRENT_LIMIT, NEO_MAX, DEFAULT_MAX_ACCELERATION);
    }

    public Command runIntake() {
        return setTargetVelocityCommand(TARGET_SPEED);
    }

    public Command idleIntake() {
        return setTargetVelocityCommand(ZERO_VELOCITY);
    }
}
