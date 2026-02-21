// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final SwerveSubsystem drivebase;
  private final CommandXboxController m_driverController;

  private final SwerveInputStream driveAngularVelocity;
  private final SwerveInputStream driveDirectAngle;
  private final Command driveFieldOrientedDirectAngle;
  private final Command driveFieldOrientedAngularVelocity;

  public RobotContainer() {
    // initialize subsystem and controller first so they are available to the command/streams
    drivebase = new SwerveSubsystem();
    m_driverController = new CommandXboxController(0);

    // build input streams and commands using initialized objects
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * -1,
        () -> m_driverController.getLeftX() * -1)
        .withControllerRotationAxis(m_driverController::getRightX)
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
        m_driverController::getRightY)
        .headingWhile(true);

    driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY());

    driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
