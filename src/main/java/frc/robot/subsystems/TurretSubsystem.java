package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax turretMotor;

    @SuppressWarnings("removal")
    public TurretSubsystem() {
        turretMotor = new SparkMax(15, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        turretMotor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setSpeed(double speed) {
        turretMotor.set(speed);
    }

    public void stop() {
        turretMotor.set(0);
    }

    public Command runTurret(double speed) {
        return Commands.startEnd(
            () -> setSpeed(speed),
            () -> stop(),
            this
        );
    }
}