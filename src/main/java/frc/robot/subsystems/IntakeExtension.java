package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents the intake extension of Fishsticks. */
@Logged
public class IntakeExtension extends SubsystemBase {
	public static final int MOTOR_ID = 0;
	public static final int CURRENT_LIMIT = (int) Units.Amps.of(20).in(Amps);

	public static final AngularVelocity FORWARD_SPEED = Units.RPM.of(250);
	public static final AngularVelocity REVERSE_SPEED = Units.RPM.of(-250);

	private final SparkMax motor;
	private final SparkMaxConfig config;

	// Stuff for logging
	double setpoint; // Requested velocity target
	double current; // Motor current draw
	double voltage; // Motor voltage applied
	double velocity; // Motor actual velocity
	boolean forwardLimit;
	boolean reverseLimit;

	public IntakeExtension(boolean isRealRobot) {
		motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
		config = new SparkMaxConfig();
		config.inverted(false);
		config.smartCurrentLimit(CURRENT_LIMIT);
		motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	private void setSpeed(AngularVelocity speed) {
		motor.set(setpoint = speed.in(RPM));
	}

	public Command setSpeedCommand(AngularVelocity speed) {
		return Commands.runOnce(() -> this.setSpeed(speed), this);
	}

	public Command deployExtension() {
		return setSpeedCommand(FORWARD_SPEED);
	}

	public Command retractExtension() {
		return setSpeedCommand(REVERSE_SPEED);
	}

	@Override
	public void periodic() {
		current = motor.getOutputCurrent();
		voltage = motor.getAppliedOutput() * motor.getBusVoltage();
		velocity = motor.getEncoder().getVelocity();
		forwardLimit = motor.getForwardLimitSwitch().isPressed();
		reverseLimit = motor.getForwardLimitSwitch().isPressed();
	}

	@Override
	public void simulationPeriodic() {
		// Pretend that everything happens instantaneously
		velocity = setpoint;
	}

	public boolean getForwardLimit() {
		return forwardLimit;
	}

	public boolean getReverseLimit() {
		return reverseLimit;
	}
}
