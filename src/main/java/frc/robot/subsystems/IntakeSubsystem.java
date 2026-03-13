package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase
{
  SparkMax intakeMotor = new SparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
  SparkMaxConfig motorConfig = new SparkMaxConfig();
  private boolean isRunning = false;

  public IntakeSubsystem()
  {
    motorConfig.inverted(IntakeConstants.MOTOR_INVERTED);
    stop();
  }

  public void start()
  {
    intakeMotor.set(IntakeConstants.INTAKE_SPEED);
    isRunning = true;
  }

  public void stop()
  {
    intakeMotor.stopMotor();
    isRunning = false;
  }

  public void toggle()
  {
    if (isRunning)
    {
      stop();
    } else
    {
      start();
    }
  }

  public boolean isRunning()
  {
    return isRunning;
  }
}
