// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (100 - 20.3) * 0.453592;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13;
  public static final double MAX_SPEED  = Units.feetToMeters(6);

  public static final class DrivebaseConstants
  {
    public static final double WHEEL_LOCK_TIME = 10;
  }

  public static class OperatorConstants
  {
    public static final double DEADBAND        = 0.4;
    public static final double TURN_CONSTANT    = 6;
  }

  // ==================== INTAKE ARM ====================
  public static final class ArmConstants {
    public static final int CAN_ID = 9;
    public static final DCMotor MOTOR = DCMotor.getNEO(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(9, 5, 3)))
            .withClosedLoopController(5, 0, 0.1)
            .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
            .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0))
            .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
            .withIdleMode(MotorMode.BRAKE);
    
    public static final ArmConfig ARM_CONFIG = 
        new ArmConfig()
            .withLength(Inches.of(21.75))
            .withMass(Pounds.of(13.5))
            .withHardLimit(Degrees.of(-0.3), Degrees.of(0.3))
            .withSoftLimits(Degrees.of(-0.25), Degrees.of(0.25))
            .withStartingPosition(Degrees.of(0))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  }

  // ==================== INTAKE WHEELS ====================
  public static final class IntakeConstants {
    public static final int CAN_ID = 10;
    public static final DCMotor MOTOR = DCMotor.getNEO(1);

    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
            .withIdleMode(MotorMode.COAST);

    public static final FlyWheelConfig INTAKE_WHEEL_CONFIG = 
        new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(0.5))
            .withSoftLimit(RPM.of(0), RPM.of(6000))
            .withTelemetry("IntakeWheel", TelemetryVerbosity.HIGH);
  }
  
  // ==================== TURRET SHOOTER ====================
  public static final class ShooterConstants {
    public static final int CAN_ID = 15;
    public static final DCMotor MOTOR = DCMotor.getNEO(1);  
    public static final Distance FLYWHEEL_DIAMETER = Inches.of(4);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
            .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withStatorCurrentLimit(Amps.of(40))
            .withIdleMode(MotorMode.COAST);
    
    public static final FlyWheelConfig FLYWHEEL_CONFIG = 
        new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withMass(Pounds.of(0.5))
            .withSoftLimit(RPM.of(0), RPM.of(6000))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH)
            .withSpeedometerSimulation(RPM.of(7500));
  }
}
