package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;

public class IntakeArmSubsystem extends SubsystemBase {

  private final SparkMax motor;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder encoder;
  private double lastKP = ArmConfig.KP;
  private double lastKI = ArmConfig.KI;
  private double lastKD = ArmConfig.KD;

  private final SysIdRoutine sysIdRoutine;

  private boolean manualMode = false;

  // Simulation
  private final SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ArmConfig.GEAR_RATIO,
          SingleJointedArmSim.estimateMOI(
            ArmConfig.ARM_LENGTH,
            ArmConfig.ARM_MASS
          ),
          ArmConfig.ARM_LENGTH,
          Math.toRadians(ArmConfig.MIN_ANGLE),
          Math.toRadians(ArmConfig.MAX_ANGLE),
          true, 
          0, 
          new double[] {}
      );

  @SuppressWarnings("removal")
  public IntakeArmSubsystem() {

    motor = new SparkMax(Constants.IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig armConfig = new SparkMaxConfig();

    armConfig
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    armConfig.encoder
      .positionConversionFactor(ArmConfig.POSITION_CONVERSION_FACTOR)
      .velocityConversionFactor(ArmConfig.VELOCITY_CONVERSION_FACTOR);

    armConfig.closedLoop
      .p(ArmConfig.KP)
      .i(ArmConfig.KI)
      .d(ArmConfig.KD)
      .outputRange(-1, 1);

    armConfig.closedLoop.maxMotion
      .cruiseVelocity(2000)
      .maxAcceleration(4000);

    armConfig.closedLoop.feedForward
      .kS(ArmConfig.KS)
      .kV(ArmConfig.KV)
      .kA(ArmConfig.KA)
      .kCos(ArmConfig.KG)
      .kCosRatio(ArmConfig.KCOS_RATIO);

    armConfig.softLimit
      .forwardSoftLimit(ArmConfig.MAX_ANGLE)
      .reverseSoftLimit(ArmConfig.MIN_ANGLE)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);

    motor.configure(
      armConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    encoder.setPosition(0);

    // SysID configuration
    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(2),
            Volts.of(6),
            Seconds.of(4)
        ),
        new SysIdRoutine.Mechanism(
            this::sysIdDrive,
            this::sysIdLog,
            this
        )
    );
  }

  // Set arm position
  public Command setAngle(double angle) {
    return runOnce(() -> {

      manualMode = false;

      double adjustedTarget = applyAdaptiveMotion(angle);

      controller.setSetpoint(
          adjustedTarget,
          ControlType.kMAXMotionPositionControl
      );
    });
  }

  // Manual override
  public Command manual(double speed) {
    return run(() -> {

      manualMode = true;
      motor.set(speed);

    }).finallyDo(() -> {

      manualMode = false;
      motor.set(0);

    });
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  // SysID voltage input
  private void sysIdDrive(Voltage volts) {
    motor.setVoltage(volts.in(Volts));
  }

  private double applyAdaptiveMotion(double targetAngle) {

    double currentAngle = getAngle();

    double gravityScale = Math.cos(Math.toRadians(currentAngle));

    double adjustment = 1 - gravityScale * 0.3;

    double delta = targetAngle - currentAngle;

    return currentAngle + delta * adjustment;
  }

  // SysID logging
  private void sysIdLog(SysIdRoutineLog log) {

    log.motor("arm")
        .voltage(
            Volts.of(
                motor.getAppliedOutput() *
                RobotController.getBatteryVoltage()
            ))
        .angularPosition(Degrees.of(encoder.getPosition()))
        .angularVelocity(DegreesPerSecond.of(encoder.getVelocity()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  @SuppressWarnings("removal")
  public void periodic() {
    // Shuffleboard tuning
    double kP = SmartDashboard.getNumber("Arm kP", ArmConfig.KP);
    double kI = SmartDashboard.getNumber("Arm kI", ArmConfig.KI);
    double kD = SmartDashboard.getNumber("Arm kD", ArmConfig.KD);

    if (kP != lastKP || kI != lastKI || kD != lastKD) {

      SparkMaxConfig pidUpdate = new SparkMaxConfig();

      pidUpdate.closedLoop
          .p(kP)
          .i(kI)
          .d(kD);

      motor.configure(
          pidUpdate,
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters
      );

      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
    }

    SmartDashboard.putNumber("Arm Angle", getAngle());
    SmartDashboard.putNumber("Arm Velocity", encoder.getVelocity());
    SmartDashboard.putBoolean("Arm Manual Mode", manualMode);

  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(
      motor.getAppliedOutput() * 
      RobotController.getBatteryVoltage()
      );

    armSim.update(0.02);

    encoder.setPosition(Math.toDegrees(armSim.getAngleRads()));
  }
}