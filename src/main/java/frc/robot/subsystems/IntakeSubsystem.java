package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeWheelMotor =
      new SparkMax(Constants.IntakeConstants.INTAKE_WHEEL_CAN_ID, MotorType.kBrushless);

  public IntakeSubsystem() {}

  public void runIntake(double speed) {
    intakeWheelMotor.set(speed);
  }

  public void stopIntake() {
    intakeWheelMotor.stopMotor();
  }

  public Command intake() {
    return runEnd(
        () -> runIntake(Constants.IntakeConstants.INTAKE_SPEED),
        () -> stopIntake());
  }

  public Command outtake() {
    return runEnd(
        () -> runIntake(Constants.IntakeConstants.OUTTAKE_SPEED),
        () -> stopIntake());
  }
}