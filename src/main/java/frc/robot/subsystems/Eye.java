package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Eye extends SubsystemBase {
    public static final double ANGLE_EAST = 0;
    public static final double ANGLE_SOUTHEAST = 45;
    public static final double ANGLE_SOUTH = 90;
    public static final double ANGLE_SOUTHWEST = 120;
    public static final double ANGLE_WEST = 180;
    public static final double ANGLE_NORTHWEST = 225;
    public static final double ANGLE_NORTH = 270; // NOTE: This is "eyer straight ahead" and as "up" as they go
    public static final double ANGLE_NORTHEAST = 315;

    private static final int TURN_SPEED_RPM = 10;
    private static final AngularVelocity TURN_LEFT = RPM.of(TURN_SPEED_RPM);
    private static final AngularVelocity TURN_RIGHT = RPM.of(-TURN_SPEED_RPM);

    private boolean isRealRobot;
    private double targetAngle = ANGLE_NORTH;
    private AngularVelocity speed = null;

    public Eye(boolean isRealRobot) {
        this.isRealRobot = isRealRobot;
    }

    /**
     * Sets the target angle for the motor.
     * 
     * @param angle
     */
    protected void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        this.speed = null;
    }

    public boolean isAtTargetAngle() {
        return true; // TODO
    }

    protected void setSpeed(AngularVelocity speed) {
        this.speed = speed;
    }

    @Override
    public void periodic() {
        // We are either set to a steady speed or heading toward a target.
        if(null != speed) {
            // motor.setSpeed(speed)
        } else {
            // motor.setTargetAngle(targetAngle)
        }
    }

    public Command getLookStraightCommand() {
        return Commands.runOnce(() -> setTargetAngle(ANGLE_NORTH));
    }

    public Command getLookLeftCommand() {
        return Commands.runOnce(() -> setTargetAngle(ANGLE_WEST));
    }

    public Command getLookRightCommand() {
        return Commands.runOnce(() -> setTargetAngle(ANGLE_EAST));
    }

    public Command getLookDownCommand() {
        return Commands.runOnce(() -> setTargetAngle(ANGLE_SOUTH));
    }

    public Command getLookDownLeftCommand() {
        return Commands.runOnce(() -> setTargetAngle(ANGLE_SOUTHWEST));
    }

    public Command getLookDownRightCommand() {
        return Commands.runOnce(() -> setTargetAngle(ANGLE_SOUTHEAST));
    }

    public Command getSpinLeftCommand() {
        return Commands.runOnce(() -> setSpeed(TURN_LEFT));
    }

    public Command getSpinRightCommand() {
        return Commands.runOnce(() -> setSpeed(TURN_RIGHT));
    }

    public Command getOscillateBetweenCommand(double angleA, double angleB) {
        return Commands.repeatingSequence(
            Commands.runOnce(() -> setTargetAngle(angleA)),
            Commands.waitUntil(this::isAtTargetAngle),
            Commands.runOnce(() -> setTargetAngle(angleB)),
            Commands.waitUntil(this::isAtTargetAngle)
        );
    }
}
