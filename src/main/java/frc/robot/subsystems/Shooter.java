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

/**
 * Represents the Fishsticks shooter. This is only the actual shooter part of the robot, and not
 * turret extension, which can be found elsewhere.
 *
 * @see Turret
 */
@Logged
public class Shooter extends SubsystemBase {
    public static final AngularVelocity TARGET_SPEED = Units.RPM.of(2000);
    public static final AngularVelocity IDLE_SPEED = Units.RPM.of(0);
    public static final int CURRENT_LIMIT = (int) Units.Amps.of(20).in(Amps);
    public static final int MOTOR_ID = 0;

    private final SparkMax motor;
    private final SparkMaxConfig config;

    // Stuff for logging
    double setpoint; // Requested velocity target
    double current; // Motor current draw
    double voltage; // Motor voltage applied
    double velocity; // Motor actual velocity

    public Shooter(boolean isRealRobot) {
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

    public Command startShooting() {
        return setSpeedCommand(TARGET_SPEED);
    }

    public Command stopShooting() {
        return setSpeedCommand(IDLE_SPEED);
    }

    @Override
    public void periodic() {
        current = motor.getOutputCurrent();
        voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        velocity = motor.getEncoder().getVelocity();
    }

    @Override
    public void simulationPeriodic() {
        // Pretend that everything happens instantaneously
        velocity = setpoint;
    }
}
