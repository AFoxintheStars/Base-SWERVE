package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.*;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeArmSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(50, 0, 0,
              DegreesPerSecond.of(90),
              DegreesPerSecondPerSecond.of(45))
          .withSimClosedLoopController(50, 0, 0,
              DegreesPerSecond.of(90),
              DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withSimFeedforward(new ArmFeedforward(0, 0, 0))
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(9)))
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25));

  private SparkMax spark =
      new SparkMax(Constants.IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);

  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private ArmConfig armCfg =
      new ArmConfig(sparkSmartMotorController)
          .withSoftLimits(Degrees.of(5), Degrees.of(80))
          .withHardLimit(Degrees.of(0), Degrees.of(90))
          .withStartingPosition(Degrees.of(0))
          .withLength(Feet.of(1.7))
          .withMass(Pounds.of(8))
          .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  private Arm arm = new Arm(armCfg);

  public IntakeArmSubsystem() {}

  public Command setAngle(Angle angle) {
    return arm.run(angle);
  }

  public Command setAngleAndStop(Angle angle, Angle tolerance) {
    return arm.runTo(angle, tolerance);
  }

  public void setAngleSetpoint(Angle angle) {
    arm.setMechanismPositionSetpoint(angle);
  }

  public Command set(double dutycycle) {
    return arm.set(dutycycle);
  }

  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}