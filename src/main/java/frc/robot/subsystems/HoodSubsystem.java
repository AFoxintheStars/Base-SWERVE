package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class HoodSubsystem extends SubsystemBase {

    private final Servo servo = new Servo(0); // PWM port
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(3); // DIO port

    // PID for position control
    private final PIDController pid = new PIDController(0.02, 0, 0);

    // Feedforward (optional but recommended)
    private final ArmFeedforward ff = new ArmFeedforward(0.2, 0.5, 0.1);

    private double targetAngleDeg = 0;

    // Limits
    private final double MIN_ANGLE = 5;
    private final double MAX_ANGLE = 100;

    public HoodSubsystem() {
        pid.setTolerance(1.0); // degrees
    }

    // ===================== CORE =====================

    public double getAngleDeg() {
        double encoderRotations = encoder.get();
        return encoderRotations * 360.0 * 0.8;
    }

    public void setTargetAngle(double angleDeg) {
        targetAngleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE);
    }

    public void updateControl() {
        double current = getAngleDeg();

        double pidOutput = pid.calculate(current, targetAngleDeg);

        double ffOutput = ff.calculate(Math.toRadians(targetAngleDeg), 0);

        double output = pidOutput + ffOutput;

        // Clamp to servo range
        output = Math.max(-1, Math.min(1, output));

        if (Math.abs(pid.getError()) < 1.0) {
            servo.set(0.5); // stop
        } else {
            servo.set(output * 0.5 + 0.5);
        }
    }

    public void setAngleDirect(Angle angle) {
    setTargetAngle(angle.in(Degrees));
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ===================== COMMANDS =====================

    public Command setAngle(Angle angle) {
        return Commands.run(
            () -> setTargetAngle(angle.in(Degrees)),
            this
        );
    }

    public Command setAngle(Supplier<Angle> supplier) {
        return Commands.run(
            () -> setTargetAngle(supplier.get().in(Degrees)),
            this
        );
    }

    public Angle getAngle() {
        return Degrees.of(getAngleDeg());
    }

    // Manual control (like old duty cycle)
    public Command setDutyCycle(Supplier<Double> supplier) {
        return Commands.run(
            () -> servo.set(supplier.get() * 0.5 + 0.5),
            this
        );
    }

    public Command setDutyCycle(double value) {
        return setDutyCycle(() -> value);
    }

    @Override
    public void periodic() {
        updateControl();
    }
}