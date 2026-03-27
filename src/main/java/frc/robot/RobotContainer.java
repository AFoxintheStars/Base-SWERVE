// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PrefeedSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretFlywheelSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  final         CommandJoystick driverJoystick = new CommandJoystick(0);
  
  private final java.util.function.Supplier<Double> throttle = () -> {
  double raw = (1.0 - driverJoystick.getRawAxis(4)) / 2.0; // 0 → 1
  return 0.1 + 0.9 * raw * raw;
  };

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final IntakeArmSubsystem intakeArm = new IntakeArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final PrefeedSubsystem prefeed = new PrefeedSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final TurretFlywheelSubsystem turretFlywheel = new TurretFlywheelSubsystem();
  public VisionSubsystem vision = new VisionSubsystem();
  // public ShooterSubsystem shooter = new ShooterSubsystem(vision);

  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> driverJoystick.getY() * -1,
                                                                  () -> driverJoystick.getX() * -1
                                                                )
                                                              .withControllerRotationAxis(() -> driverJoystick.getZ() * -1)
                                                              .deadband(OperatorConstants.DEADBAND)
                                                              .scaleTranslation(0.8)
                                                              .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverJoystick.getRawAxis(1),
                                                                                               () -> driverJoystick.getRawAxis(2))
                                                             .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative 
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverJoystick.getY(),
                                                                        () -> -driverJoystick.getX())
                                                                    .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverJoystick.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    intake.setDefaultCommand(intake.set(0));

    turretFlywheel.setDefaultCommand(turretFlywheel.setDutyCycle(0));

    intakeArm.setDefaultCommand(intakeArm.set(0));

    turret.setDefaultCommand(
        turret.manualControl(() -> driverJoystick.getHID().getPOV())
    );

    hood.setDefaultCommand(
        hood.manualControl(() -> driverJoystick.getHID().getPOV())
    );

    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putNumber("Robot Speed (m/s)", drivebase.getSpeedMetersPerSecond());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverJoystick.button(1).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverJoystick.button(3).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverJoystick.button(4).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );
    }

      driverJoystick.button(3).onTrue(drivebase.zeroGyroWithAllianceCommand());

      driverJoystick.button(5).whileTrue(turretFlywheel.setDutyCycle(0.9));

      driverJoystick.button(5).whileTrue(
          new WaitCommand(0.5).andThen(prefeed.runPrefeed(0.8))
      );

      driverJoystick.button(6).whileTrue(prefeed.runPrefeed(-0.6));

      driverJoystick.button(10).whileTrue(intakeArm.set(0.5));
      driverJoystick.button(9).whileTrue(intakeArm.set(-0.3));
           
      driverJoystick.button(1).whileTrue(intake.set(0.8));
      driverJoystick.button(2).whileTrue(intake.set(-0.8));

      driverJoystick.button(11).whileTrue(climber.climbUp());
      driverJoystick.button(12).whileTrue(climber.climbDown());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
