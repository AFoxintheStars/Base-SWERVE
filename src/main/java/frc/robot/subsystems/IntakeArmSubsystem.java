	// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeArmSubsystem extends SubsystemBase {

  private Arm arm;

  public IntakeArmSubsystem() {
    SmartMotorControllerConfig smcConfig = Constants.ArmConstants.SMC_CONFIG
    .withSubsystem(this);

  SmartMotorController smc = new SparkWrapper(
      new SparkMax(ArmConstants.CAN_ID, MotorType.kBrushless),
      ArmConstants.MOTOR,
      smcConfig
  );

    ArmConfig armConfig = Constants.ArmConstants.ARM_CONFIG
      .withSmartMotorController(smc);

      this.arm = new Arm(armConfig);
  }

  // ==================== COMMANDS ====================
  
  public Command runToAngle(Angle target) {
    return arm.runTo(target, Degrees.of(3));
  }

  public Command holdAngle(Angle target) {
    return arm.run(target);
  }

  public Command stow() {
    return arm.runTo(Degrees.of(0), Degrees.of(3));
  }

  // ==================== ACCESSORS ====================
  
  public Arm getArm() {
    return arm;
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