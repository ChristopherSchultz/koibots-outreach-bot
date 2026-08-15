package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A base class for fire-and-forget spinning subsystems like shooters and flywheels.
 */
@Logged
public class SimpleSpinningSubsystem extends SubsystemBase {
    public static final AngularVelocity ZERO_VELOCITY = Units.RPM.of(0);

    public static final AngularVelocity NEO_MAX = RPM.of(5676);
    public static final AngularVelocity NEO_VORTEX_MAX = RPM.of(6784);
    public static final AngularVelocity NEO_550_MAX = RPM.of(11000);
    public static final AngularAcceleration DEFAULT_MAX_ACCELERATION = RPM.per(Second).of(500);
    public static final Current DEFAULT_CURRENT_LIMIT = Amps.of(20);

    private final boolean isRealRobot;

    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final SparkClosedLoopController controller;
    // For simulation
    private static final double SIMULATION_PERIOD_SECONDS = 0.020;
    private final SparkMaxSim motorSim;
    private final FlywheelSim flywheelSim;

    private final AngularVelocity maxVelocity;
    private final AngularVelocity velocityTolerance;

    private AngularVelocity targetVelocity = ZERO_VELOCITY;

    // Stuff for logging
    private double targetRPM;
    private double measuredRPM; // Motor actual velocity
    private double current; // Motor current draw
    private double voltage; // Motor voltage applied
    private final Timer timeToTarget = new Timer();
    private boolean reachedTarget = true;

    public SimpleSpinningSubsystem(boolean isRealRobot, int motorId, Current currentLimit, AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration, AngularVelocity velocityTolerance) {
        this.isRealRobot = isRealRobot;
        this.maxVelocity = maxVelocity;
        this.velocityTolerance = velocityTolerance;

        motor = new SparkMax(motorId, MotorType.kBrushless);
        if (isRealRobot) {
            motorSim = null;
            flywheelSim = null;
        } else {
            DCMotor motorModel = DCMotor.getNEO(1);
            motorSim = new SparkMaxSim(motor, motorModel);
            flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorModel, 0.001, // moment of inertia,
                                                                                                 // kg·m²
                    1.0), // gearing
                    motorModel, 0.0);
        }

        controller = motor.getClosedLoopController();
        config = new SparkMaxConfig();
        config.inverted(false);
        config.smartCurrentLimit((int) currentLimit.in(Amps));
        // TODO: P/I/D control from subclass
        config.closedLoop.p(0.0001) // TODO: Very conservative starting value
                .i(0).d(0);

        // TODO: Allow subclass to specify feedforward values
        config.closedLoop.feedForward.kV(0.0021); // TODO: Very conservative starting value

        // NOTE: maxAcceleration() wants RPM/s not RPS/S
        config.closedLoop.maxMotion.maxAcceleration(60 * maxAcceleration.in(Units.RotationsPerSecondPerSecond));

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setTargetVelocity(AngularVelocity velocity) {
        double targetRPM = velocity.in(RPM);

        if (Math.abs(targetRPM) > maxVelocity.in(RPM)) {
            throw new IllegalArgumentException("Requested velocity exceeds maximum velocity");
        }

        targetVelocity = velocity;
        this.targetRPM = targetRPM;
        reachedTarget = false;
        timeToTarget.restart();
        controller.setSetpoint(targetRPM, ControlType.kMAXMotionVelocityControl);
    }

    /**
     * Implements a passive stop. No power will be applied in order to actively stop the mechanism.
     */
    protected void stop() {
        controller.setSetpoint(0, ControlType.kDutyCycle);
        targetVelocity = ZERO_VELOCITY;
        targetRPM = 0;
        reachedTarget = true; // Not strictly true, but we also don't care.
        timeToTarget.stop();
    }

    protected AngularVelocity getTargetVelocity() {
        return targetVelocity;
    }

    protected AngularVelocity getVelocity() {
        return RPM.of(measuredRPM);
    }

    protected boolean isRealRobot() {
        return isRealRobot;
    }

    protected Command stopCommand() {
        return Commands.runOnce(() -> stop());
    }

    protected Command setTargetVelocityCommand(final AngularVelocity velocity) {
        return Commands.runOnce(() -> setTargetVelocity(velocity), this);
    }

    public boolean isAtTargetVelocity() {
        return reachedTarget;
    }

    public static boolean isWithinTolerances(double a, double b, double tolerance) {
        return Math.abs(a - b) <= tolerance;
    }

    @Override
    public void periodic() {
        current = motor.getOutputCurrent();
        voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        measuredRPM = motor.getEncoder().getVelocity();

        if (!reachedTarget && isWithinTolerances(measuredRPM, targetRPM, velocityTolerance.in(RPM))) {
            timeToTarget.stop();
            reachedTarget = true;
        }
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();

        flywheelSim.setInputVoltage(motorSim.getAppliedOutput() * batteryVoltage);

        flywheelSim.update(SIMULATION_PERIOD_SECONDS);

        motorSim.iterate(flywheelSim.getAngularVelocityRPM(), batteryVoltage, SIMULATION_PERIOD_SECONDS);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrent() {
        return current;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getMeasuredRPM() {
        return measuredRPM;
    }

    public double getTimeToTarget() {
        return timeToTarget.get();
    }
}
