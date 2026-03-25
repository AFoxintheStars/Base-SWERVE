// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {

    private FlyWheel intakeWheel;

    public IntakeSubsystem() {
        SmartMotorControllerConfig smcConfig = Constants.IntakeConstants.SMC_CONFIG
        .withSubsystem(this);

        SmartMotorController smc = new SparkWrapper(
            new SparkMax(Constants.IntakeConstants.CAN_ID, MotorType.kBrushless),
            Constants.IntakeConstants.MOTOR,
             smcConfig
        );

        FlyWheelConfig intakeWheelConfig = Constants.IntakeConstants.INTAKE_WHEEL_CONFIG
        .withSmartMotorController(smc);

        this.intakeWheel = new FlyWheel(intakeWheelConfig);
    }

    // ==================== COMMANDS ====================

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return intakeWheel.getSpeed();
    }
    
    /**
     * Set the intake wheel velocity setpoint.
     *
     * @param speed Speed to set
     */
    public void setVelocitySetpoint(AngularVelocity speed) {
        intakeWheel.setMechanismVelocitySetpoint(speed);
    }
    
    /**
     * Set the intake wheel velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return intakeWheel.run(speed);
    }

    /**
     * Set the dutycycle of the intake wheel.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return intakeWheel.set(dutyCycle);
    }

    // ==================== ACCESSORS ====================
  
    public FlyWheel getFlyWheel() {
    return intakeWheel;
    }

    @Override
    public void periodic() {
    intakeWheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
    intakeWheel.simIterate();
    }
}
