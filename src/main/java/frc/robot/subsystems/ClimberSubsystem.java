package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor;
    private final SparkMaxConfig config;

    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 80.0 * 10;

    private static final double CLIMB_SPEED = 0.8;
    private static final double DESCEND_SPEED = -0.7;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(11, MotorType.kBrushless);
        config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.inverted(false);

        config.softLimit
            .forwardSoftLimit(MAX_POSITION)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(MIN_POSITION)
            .reverseSoftLimitEnabled(true);

        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Optional: zero encoder at startup
        climberMotor.getEncoder().setPosition(0);
    }

    // ===============================
    // BASIC CONTROL METHODS
    // ===============================

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void stop() {
        climberMotor.set(0);
    }

    public double getPosition() {
        return climberMotor.getEncoder().getPosition();
    }

    // ===============================
    // COMMANDS
    // ===============================

    public Command climbUp() {
        return run(() -> {
            if (getPosition() < MAX_POSITION) {
                setSpeed(CLIMB_SPEED);
            } else {
                stop();
            }
        }).finallyDo(this::stop);
    }

    public Command climbDown() {
        return run(() -> {
            if (getPosition() > MIN_POSITION) {
                setSpeed(DESCEND_SPEED);
            } else {
                stop();
            }
        }).finallyDo(this::stop);
    }
}