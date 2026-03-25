package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

        private final Pivot turret;
        private final SmartMotorController smc;

        private final DutyCycleEncoder encoder1 = new DutyCycleEncoder(1);
        private final DutyCycleEncoder encoder2 = new DutyCycleEncoder(0);

        private EasyCRT solver;

        public TurretSubsystem() {
                SmartMotorControllerConfig smcConfig = Constants.TurretConstants.SMC_CONFIG
                .withSubsystem(this);

                smc = new SparkWrapper(
                new SparkMax(Constants.TurretConstants.CAN_ID, MotorType.kBrushless),
                Constants.TurretConstants.MOTOR,
                smcConfig
                );

                PivotConfig turretConfig = new PivotConfig(smc)
                        .withStartingPosition(Degrees.of(0))
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH);

                this.turret = new Pivot(turretConfig);

                // ================= CRT SETUP =================

                var easyCrtConfig = new EasyCRTConfig(
                        () -> Rotations.of(encoder1.get()),
                        () -> Rotations.of(encoder2.get())
                )
                .withCommonDriveGear(1.0, 200, 19, 21)
                .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0))
                .withMechanismRange(Rotations.of(-1.0), Rotations.of(2.0))
                .withMatchTolerance(Rotations.of(0.05))
                .withAbsoluteEncoderInversions(false, false);

                solver = new EasyCRT(easyCrtConfig);

                solver.getAngleOptional().ifPresent(angle -> {
                smc.setPosition(angle);
                });
        }

        public Command setAngle(Angle angle) {
                return turret.setAngle(angle);
        }

        public void setAngleDirect(Angle angle) {
                smc.setPosition(angle);
        }

        public Command setAngle(Supplier<Angle> angleSupplier) {
                return turret.setAngle(angleSupplier);
        }

        public Angle getAngle() {
                return turret.getAngle();
        }

        public Command sysId() {
                return turret.sysId(
                        Volts.of(4.0), 
                        Volts.per(Second).of(0.5),
                        Seconds.of(8.0)
                );
        }

        public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
                return turret.set(dutyCycleSupplier);
        }

        public Command setDutyCycle(double dutyCycle) {
                return turret.set(dutyCycle);
        }

        @Override
        public void periodic() {
                turret.updateTelemetry();
                // System.out.println("CRT Status: " + solver.getLastStatus());
                // System.out.println("CRT Error: " + solver.getLastErrorRotations());
        }

        @Override
        public void simulationPeriodic() {
                turret.simIterate();
        }
}