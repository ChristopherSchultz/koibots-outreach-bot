package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Eye extends SubsystemBase {
    public static final double ANGLE_EAST = 0;
    public static final double ANGLE_NORTHEAST = 45;
    public static final double ANGLE_NORTH = 90; // NOTE: This is "eye straight ahead" and as "up" as it goes
    public static final double ANGLE_NORTHWEST = 135;
    public static final double ANGLE_WEST = 180;
    public static final double ANGLE_SOUTHWEST = 225;
    public static final double ANGLE_SOUTH = 270;
    public static final double ANGLE_SOUTHEAST = 315;

    public static final double ANGLE_TOLERANCE = 2.0;

    private boolean isRealRobot;
    private double targetAngle = ANGLE_NORTH;

    private final Servo servo;

    protected Eye(boolean isRealRobot, int pwmChannel) {
        this.isRealRobot = isRealRobot;
        servo = new Servo(pwmChannel);

        setTargetAngle(targetAngle);
    }

    /**
     * Sets the target angle for the motor.
     *
     * @param angle
     */
    protected void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        servo.setPosition(targetAngle / 270.0); // NOTE: angle is being adjusted into 0 - 270º working space
    }

    /**
     * Checks to see if the Eye is at the target angle.
     */
    public boolean isAtTargetAngle() {
        return Math.abs(servo.getPosition() - targetAngle) <= ANGLE_TOLERANCE;
    }

    protected Command lookAtCommand(double angle) {
        return Commands.runOnce(() -> setTargetAngle(angle), this);
    }

    protected Command lookAtAndWaitCommand(double angle) {
        return lookAtCommand(angle)
            .andThen(Commands.waitUntil(this::isAtTargetAngle))
            ;
    }

    public Command getLookStraightCommand() {
        return lookAtAndWaitCommand(ANGLE_NORTH);
    }

    public Command getLookLeftCommand() {
        return lookAtAndWaitCommand(ANGLE_WEST);
    }

    public Command getLookRightCommand() {
        return lookAtAndWaitCommand(ANGLE_EAST);
    }

    public Command getLookDownCommand() {
        return lookAtAndWaitCommand(ANGLE_SOUTH);
    }

    public Command getLookDownLeftCommand() {
        return lookAtAndWaitCommand(ANGLE_SOUTHWEST);
    }

    public Command getLookDownRightCommand() {
        return lookAtAndWaitCommand(ANGLE_SOUTHEAST);
    }

    public Command getOscillateBetweenCommand(double angleA, double angleB) {
        return Commands.repeatingSequence(
                lookAtAndWaitCommand(angleA),
                lookAtAndWaitCommand(angleB))
                ;
    }
}
