// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class TurretFlywheelSubsystem extends SubsystemBase {

    private FlyWheel shooter;
    private SmartMotorController smc;

    public TurretFlywheelSubsystem() {
        SmartMotorControllerConfig smcConfig = Constants.ShooterConstants.SMC_CONFIG
        .withSubsystem(this);

         smc = new SparkWrapper(
            new SparkMax(Constants.ShooterConstants.CAN_ID, MotorType.kBrushless),
            Constants.ShooterConstants.MOTOR,
             smcConfig
        );

        FlyWheelConfig shooterConfig = Constants.ShooterConstants.FLYWHEEL_CONFIG
        .withSmartMotorController(smc);

        this.shooter = new FlyWheel(shooterConfig);
    }

    // ==================== COMMANDS ====================

    /**
     * Gets the current velocity of the shooter.
     *
     * @return Shooter velocity.
     */
    public AngularVelocity getVelocity() {
        return shooter.getSpeed();
    }
    
    /**
     * Set the shooter velocity setpoint.
     *
     * @param speed Speed to set
     */
    public void setVelocitySetpoint(AngularVelocity speed) {
        shooter.setMechanismVelocitySetpoint(speed);
    }
    
    /**
     * Set the shooter velocity.
     *
     * @param speed Speed to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command setVelocity(AngularVelocity speed) {
        return shooter.run(speed);
    }

    /**
     * Set the dutycycle of the shooter.
     *
     * @param dutyCycle DutyCycle to set.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
     */
    public Command set(double dutyCycle) {
        return shooter.set(dutyCycle);
    }

      public Command setVelocity(Supplier<AngularVelocity> speed)
    {
        return shooter.setSpeed(speed);
    }

     public Command setDutyCycle(Supplier<Double> dutyCycle)
    {
        return shooter.set(dutyCycle);
    }

    // ==================== ACCESSORS ====================
  
    public FlyWheel getFlyWheel() {
    return shooter;
    }

    @Override
    public void periodic() {
    shooter.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
    shooter.simIterate();
    }

    public Command setRPM(LinearVelocity speed)
    {
        return shooter.setSpeed(RotationsPerSecond.of(speed.in(MetersPerSecond) / Constants.ShooterConstants.FLYWHEEL_DIAMETER.times(Math.PI).in(Meters)));
    }


    public void setRPMDirect(LinearVelocity speed)
    {
        smc.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / Constants.ShooterConstants.FLYWHEEL_DIAMETER.times(Math.PI).in(Meters)));
    }

}
