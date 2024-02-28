// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.DriveToNoteCommand;
import frc.robot.commands.miscellaneous.RumbleCommand;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.commands.subsystems.drive.DriveToPoseTrajPIDCommand;
import frc.robot.commands.subsystems.drive.TurnToAngleCommand;
import frc.robot.commands.subsystems.intake.IntakeAdjustmentCommand;
import frc.robot.commands.subsystems.intake.IntakeCommand;
import frc.robot.commands.subsystems.shooter.ShooterCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.PresetManager;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.utils.CommandButtonController;
import java.io.File;

public class RobotContainer {

  int m_angle = 180;
  int m_length = 0;

  // DriveSubsystem m_drive = new DriveSubsystem();
  Dashboard m_dashboard;

  // DriveSubsystem m_drive = new DriveSubsystem();
  public final SwerveSubsystem m_drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve_goose"));

  // public final ArmSubsystem m_arm = new ArmSubsystem();
  public final ArmRotationSubsystem m_armRot = new ArmRotationSubsystem();
  public final ArmExtensionSubsystem m_armExt = new ArmExtensionSubsystem();

  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final LEDSubsystem m_leds = new LEDSubsystem();
  public PresetManager m_presetStorage = new PresetManager();

  private final CommandXboxController m_xboxController =
      new CommandXboxController(Constants.OperatorConstants.kXboxControllerPort);

  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.OperatorConstants.kTestControllerPort);

  private final CommandButtonController m_buttonBox =
      new CommandButtonController(Constants.OperatorConstants.kButtonBoxPort);

  private boolean isSlowDrive = false;
  private double driveSpeedFactor = 1;
  private double rotationSpeedFactor = 1;

  private class RotateSlowCommand extends RunCommand {
    public RotateSlowCommand(boolean clockwise) {
      super(
          () ->
              m_drivebase.drive(
                  new Translation2d(),
                  clockwise
                      ? -Constants.OperatorConstants.kIncrementalRotationSpeed
                      : Constants.OperatorConstants.kIncrementalRotationSpeed,
                  true));
    }
  }

  Command driveFieldOrientedAnglularVelocity;

  public RobotContainer() {

    configureBindings();
    configureDefaultCommands();

    m_dashboard =
        new Dashboard(m_drivebase, m_armRot, m_armExt, m_intake, m_shooter, m_presetStorage);
  }

  private void configureBindings() {

    // Rotation & Extension Adjustment
    m_testController
        .b()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg(m_armRot.getRotDegrees() + 1)));
    m_testController
        .a()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg(m_armRot.getRotDegrees() - 1)));
    m_testController
        .y()
        .onTrue(new InstantCommand(() -> m_armExt.setLengthInches(m_armExt.getExtInches() + 1)));
    m_testController
        .x()
        .onTrue(new InstantCommand(() -> m_armExt.setLengthInches(m_armExt.getExtInches() - 1)));

    // Store Angle/Extension + Shooter Speed to Amp
    m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_presetStorage.set(Preset.kShootAmp)));

    // Store Angle/Extension + Shooter Speed to ShootSpeaker 0m
    m_buttonBox
        .button_3()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Preset.kShootSpeaker_0m)));

    // Activate DriveToNote Intaking (Documentation Order? To Be Tested)
    m_buttonBox
        .button_4()
        .toggleOnTrue(
            new DriveToNoteCommand(m_drivebase)
                .deadlineWith(
                    new SequentialCommandGroup(
                        new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup),
                        new IntakeCommand(m_intake)
                            .andThen(new IntakeAdjustmentCommand(m_intake))
                            .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)))));

    // Move Arm to Stow
    m_buttonBox.button_6().onTrue(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow));

    // Activate DriveToNote Intaking (Ned's Order)
    m_buttonBox
        .button_7()
        .toggleOnTrue(
            new SequentialCommandGroup(
                    new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup),
                    new IntakeCommand(m_intake)
                        .andThen(new IntakeAdjustmentCommand(m_intake))
                        .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)))
                .deadlineWith(new DriveToNoteCommand(m_drivebase)));

    // Basic Shooting from The Speaker
    m_buttonBox
        .button_8()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Preset.kShootSpeaker_0m, () -> 0.5)
                .andThen(new ShooterCommand(m_intake, m_shooter, () -> Preset.kShootSpeaker_0m)));

    // Zero The Arm Extension (to be used if reset didn't happen at trobot activation)
    m_buttonBox.button_9().onTrue(new InstantCommand(() -> m_armExt.reset()));

    // Triggers Intake (set arm + start intaking))
    m_xboxController
        .leftBumper()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup)
                .andThen(
                    new IntakeCommand(m_intake)
                        .andThen(new IntakeAdjustmentCommand(m_intake))
                        .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow))));

    // Triggers shooting to stored preset settings
    m_xboxController
        .rightBumper()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> m_presetStorage.get(), () -> 0.5)
                .andThen(new ShooterCommand(m_intake, m_shooter, () -> m_presetStorage.get()))
                .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)));

    // TODO: Assign commands to whatever the controller paddles are mapped to on the controller
    // hardware
    // m_xboxController.x().whileTrue(new RotateSlowCommand(false));
    // m_xboxController.b().whileTrue(new RotateSlowCommand(true));

    m_xboxController.y().onTrue(new InstantCommand(() -> toggleSlowDrive()));

    // TODO: Assign commands to whatever the controller paddles are mapped to on the controller
    // hardware
    m_xboxController.x().whileTrue(new RotateSlowCommand(false));
    m_xboxController.b().whileTrue(new RotateSlowCommand(true));

    m_xboxController
        .start()
        .toggleOnTrue(
            new RumbleCommand(m_xboxController.getHID(), 1, 5, 0.5, 0.25, RumbleType.kBothRumble));
  }

  private void configureDefaultCommands() {

    // AbsoluteDriveAdv closedAbsoluteDriveAdv =
    //     new AbsoluteDriveAdv(
    //         m_drivebase,
    //         () ->
    //             MathUtil.applyDeadband(
    //                 -m_xboxController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //         () ->
    //             MathUtil.applyDeadband(
    //                 m_xboxController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //         () ->
    //             MathUtil.applyDeadband(
    //                 -m_xboxController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    //         m_xboxController.getHID()::getYButtonPressed,
    //         m_xboxController.getHID()::getAButtonPressed,
    //         m_xboxController.getHID()::getXButtonPressed,
    //         m_xboxController.getHID()::getBButtonPressed);

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
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1) * driveSpeedFactor,
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1) * driveSpeedFactor,
            () -> -m_xboxController.getRawAxis(4) * rotationSpeedFactor);

    driveFieldOrientedAnglularVelocity.setName("Drive Field Oriented Anglular Velocity Command");

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
    // Command controlArm =
    //     m_arm.controlCommand(
    //         () -> MathUtil.applyDeadband(m_testController.getLeftY(), 0.1),
    //         () -> MathUtil.applyDeadband(m_testController.getLeftX(), 0.1));

    // m_arm.setDefaultCommand(controlArm);
  }

  public void toggleSlowDrive() {
    driveSpeedFactor = isSlowDrive ? 1 : Constants.OperatorConstants.kSlowDriveSpeedFactor;
    rotationSpeedFactor = isSlowDrive ? 1 : Constants.OperatorConstants.kSlowRotationSpeedFactor;

    isSlowDrive = !isSlowDrive;
  }

  public Command getAutonomousCommand() {
    switch (m_dashboard.getSelectedAuto()) {
      default:
        return new RumbleCommand(
            m_xboxController.getHID(), 1, driveSpeedFactor, RumbleType.kBothRumble);

      case EXIT:
        // return new DriveToPoseCommand(
        //     m_drivebase, new Pose2d(0, 0, Rotation2d.fromDegrees(90)), false);
        return new TurnToAngleCommand(m_drivebase, () -> new Pose2d(new Translation2d(), Rotation2d.fromDegrees(45)), isSlowDrive);

      case PRINT:
        return Commands.print("Print Auto Command");

      case DO_NOTHING:
        return Commands.none();

      case TRAJECTORY_DTP:
        return new DriveToPoseTrajPIDCommand(
            m_drivebase, () -> new Pose2d(15, 5.5, Rotation2d.fromDegrees(180)), false);
    }
  }
}
