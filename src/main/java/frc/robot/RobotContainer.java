// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveConstants.IOControlsConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.utils.CommandButtonController;
import java.io.File;

public class RobotContainer {
  // 0.2
  DriveSubsystem m_drive = new DriveSubsystem();
  ShooterSubsystem m_shooter = new ShooterSubsystem();
  Dashboard m_dashboard;

  // DriveSubsystem m_drive = new DriveSubsystem();
  public final SwerveSubsystem m_drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public final ArmSubsystem m_arm = new ArmSubsystem();

  private final CommandXboxController m_xboxController =
      new CommandXboxController(IOControlsConstants.kXboxControllerPort);

  private final CommandXboxController m_testController =
      new CommandXboxController(IOControlsConstants.kTestControllerPort);

  private final CommandButtonController m_buttonBox =
      new CommandButtonController(IOControlsConstants.kButtonBoxPort);

  Command driveFieldOrientedAnglularVelocity;

  // Initialized here because speed factors are created in constants
  private final Command slowDriveCommand =
      m_drivebase.driveCommand(
          () ->
              MathUtil.applyDeadband(
                  -m_xboxController.getLeftY() * Constants.OperatorConstants.kSlowDriveSpeedFactor,
                  0.1),
          () ->
              MathUtil.applyDeadband(
                  -m_xboxController.getLeftX() * Constants.OperatorConstants.kSlowDriveSpeedFactor,
                  0.1),
          () ->
              -m_xboxController.getRawAxis(4)
                  * Constants.OperatorConstants.kSlowRotationSpeedFactor);

  private class RotateSlowCommand extends RunCommand {
    public RotateSlowCommand(boolean clockwise) {
      super(
          () ->
              m_drivebase.driveCommand(
                  () -> 0,
                  () -> 0,
                  () ->
                      clockwise
                          ? -Constants.OperatorConstants.kIncrementalRotationSpeed
                          : Constants.OperatorConstants.kIncrementalRotationSpeed),
          m_drivebase);
    }
  }

  public RobotContainer() {

    configureBindings();
    configureDefaultCommands();

    m_dashboard = new Dashboard(m_drivebase, m_arm);
  }

  private void configureBindings() {
    /** Button Box Bindings */
    m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(-103)));
    m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(20.0)));

    m_buttonBox.button_3().onTrue(new InstantCommand(() -> m_arm.setLengthInches(0)));
    m_buttonBox.button_4().onTrue(new InstantCommand(() -> m_arm.setLengthInches(10)));

    /** Test Controller Buttons * */
    m_testController
        .a()
        .onTrue(new InstantCommand(() -> m_arm.reset())); // Start Zeroing of the arm

    m_testController.rightStick().onTrue(new InstantCommand(() -> toggleSlowDrive()));

    // TODO: Assign commands to whatever the controller paddles are mapped to on the controller
    // hardware
    m_xboxController.x().whileTrue(new RotateSlowCommand(false));
    m_xboxController.b().whileTrue(new RotateSlowCommand(true));
  }

  private void configureDefaultCommands() {

    AbsoluteDriveAdv closedAbsoluteDriveAdv =
        new AbsoluteDriveAdv(
            m_drivebase,
            () ->
                MathUtil.applyDeadband(
                    -m_xboxController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    m_xboxController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
            () ->
                MathUtil.applyDeadband(
                    -m_xboxController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
            () -> m_xboxController.b().getAsBoolean(),
            () -> m_xboxController.x().getAsBoolean(),
            () -> m_xboxController.y().getAsBoolean(),
            () -> m_xboxController.a().getAsBoolean());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle =
        m_drivebase.driveCommand(
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
            () -> m_xboxController.getRightX(),
            () -> m_xboxController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    driveFieldOrientedAnglularVelocity =
        m_drivebase.driveCommand(
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
            () -> -m_xboxController.getRawAxis(4));

    driveFieldOrientedAnglularVelocity.setName("Drive Field Oriented Anglular Velocity Command");
    slowDriveCommand.setName("Slow Drive Field Oriented Anglular Velocity Command");

    Command driveFieldOrientedDirectAngleSim =
        m_drivebase.simDriveCommand(
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
            () -> -m_xboxController.getRawAxis(4));

    m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedAnglularVelocity
            : driveFieldOrientedDirectAngleSim);
    //  !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle :
    // driveFieldOrientedDirectAngleSim);

    // Manual Arm Control on test controller
    // X-> Rotation
    // Y-> Extention
    Command controlArm =
        m_arm.controlCommand(
            () -> MathUtil.applyDeadband(m_testController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(m_testController.getLeftX(), 0.1));

    m_arm.setDefaultCommand(controlArm);
  }

  public void toggleSlowDrive() {
    switch (m_drivebase.getDefaultCommand().getName()) {
      case "Drive Field Oriented Anglular Velocity Command":
        m_drivebase.setDefaultCommand(slowDriveCommand);
        break;

      case "Slow Drive Field Oriented Anglular Velocity Command":
        m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        break;
    }
  }

  public Command getAutonomousCommand() {
    switch (m_dashboard.getSelectedAuto()) {
      default:
        return Commands.print("Print Auto Command");

      case EXIT:
        // return new DriveToPoseCommand(m_drivebase, new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
        return m_drivebase.driveToPose(new Pose2d(3, 0, Rotation2d.fromDegrees(0)));

      case PRINT:
        return Commands.print("Print Auto Command");

      case DO_NOTHING:
        return Commands.none();
    }
  }
}
