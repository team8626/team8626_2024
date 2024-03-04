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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.DriveToNoteCommand;
import frc.robot.commands.miscellaneous.RumbleCommand;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.commands.subsystems.drive.DriveToPoseCommand;
import frc.robot.commands.subsystems.drive.DriveToPoseTrajPIDCommand;
import frc.robot.commands.subsystems.drive.TurnToAngleCommand;
import frc.robot.commands.subsystems.intake.EjectIntakeCommand;
import frc.robot.commands.subsystems.intake.IntakeAdjustmentCommand;
import frc.robot.commands.subsystems.intake.IntakeCommand;
import frc.robot.commands.subsystems.shooter.SpinAndShootCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.PresetManager;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.utils.CommandButtonController;
import java.io.File;

public class RobotContainer {

  // DriveSubsystem m_drive = new DriveSubsystem();
  Dashboard m_dashboard;

  // DriveSubsystem m_drive = new DriveSubsystem();
  public final SwerveSubsystem m_drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // public final ArmSubsystem m_arm = new ArmSubsystem();
  public final ArmRotationSubsystem m_armRot = new ArmRotationSubsystem();
  public final ArmExtensionSubsystem m_armExt = new ArmExtensionSubsystem();

  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

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
        new Dashboard(
            m_drivebase, m_armRot, m_armExt, m_intake, m_shooter, m_climber, m_presetStorage);
  }

  private void configureBindings() {

    // ---------------------------------------- MAIN CONTROLLER -------------------------
    // ----------------------------------------------------------------------------------
    //

    // ---------------------------------------- Triggers Intake (set arm + start intaking))
    m_xboxController
        .leftBumper()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup)
                .andThen(
                    new IntakeCommand(m_intake)
                        .andThen(new IntakeAdjustmentCommand(m_intake))
                        .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow))));

    // ---------------------------------------- Triggers shooting to stored preset settings
    // m_xboxController
    //     .rightBumper()
    //     .toggleOnTrue(
    //         new SetArmCommand(m_armRot, m_armExt, () -> m_presetStorage.get(), () -> 0.5)
    //             .andThen(new ShooterCommand(m_intake, m_shooter, () -> m_presetStorage.get()))
    //             .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)));
    m_xboxController
        .rightBumper()
        .toggleOnTrue(
            new SpinAndShootCommand(
                    m_intake, m_shooter, m_armRot, m_armExt, () -> m_presetStorage.get())
                .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)));

    // ---------------------------------------- Y
    //                                          Eject
    m_xboxController.y().toggleOnTrue(new EjectIntakeCommand(m_intake));

    // ---------------------------------------- POV
    //                                              Robot angle
    m_xboxController
        .povUp()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase, () -> new Pose2d(new Translation2d(), new Rotation2d(0)), true));
    m_xboxController
        .povLeft()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () -> new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                true));
    m_xboxController
        .povDown()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase, () -> new Pose2d(new Translation2d(), new Rotation2d(Math.PI)), true));
    m_xboxController
        .povRight()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () -> new Pose2d(new Translation2d(), new Rotation2d(-Math.PI / 2)),
                true));

    m_xboxController.rightTrigger().onTrue(new InstantCommand(() -> toggleSlowDrive()));

    // ---------------------------------------- TEST CONTROLLER -------------------------
    // ----------------------------------------------------------------------------------
    //
    // ---------------------------------------- Set to Angle Testing (Test Controller)
    //                                          A/B Rotation
    //                                          X/Y Extension
    m_testController
        .y()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg((m_armRot.getRotDegrees() - 5))));
    m_testController
        .a()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg((m_armRot.getRotDegrees() + 5))));
    m_testController
        .b()
        .onTrue(new InstantCommand(() -> m_armExt.setLengthInches((m_armExt.getExtInches() + 1))));
    m_testController
        .x()
        .onTrue(new InstantCommand(() -> m_armExt.setLengthInches((m_armExt.getExtInches() - 1))));

    // ---------------------------------------- Start/Stop Intake
    m_testController
        .leftBumper()
        .toggleOnTrue(
            new StartEndCommand(
                () -> m_intake.start(IntakeConstants.kSpeed_Shoot), () -> m_intake.stop()));

    // ---------------------------------------- Start/Stop Shooter
    m_testController
        .rightBumper()
        .toggleOnTrue(
            new StartEndCommand(
                () -> m_shooter.start(Preset.kShootSpeaker_0m), () -> m_shooter.stop()));

    // ---------------------------------------- Climb
    m_testController
        .leftTrigger()
        .whileTrue(new InstantCommand(() -> m_climber.setPower(-1.0)))
        .onFalse(new InstantCommand(() -> m_climber.setPower(0)));

    m_testController
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> m_climber.setPower(1.0)))
        .onFalse(new InstantCommand(() -> m_climber.setPower(0)));

    // m_testController.rightBumper().toggleOnTrue(m_shooter.startCmd());

    // ---------------------------------------- BUTTON BOX ------------------------------
    //
    //           +-----------------+
    //           |  1  |  4  |  7  |
    //           |-----+-----+-----|
    //           |  2  |  5  |  8  |
    //           |-----+-----+-----|
    //           |  3  |  6  |  9  |
    //           +-----------------+
    //
    // ----------------------------------------------------------------------------------
    //
    // ---------------------------------------- BUTTON 1
    //                                          Preset: ShootSpeaker Long Distance
    m_buttonBox
        .button_1()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Preset.kShootSpeaker_2m)));

    // ---------------------------------------- BUTTON 2
    //                                          Preset:  Amp
    m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_presetStorage.set(Preset.kShootAmp)));

    // ---------------------------------------- BUTTON 3
    //                                          Preset: ShootSpeaker 0m
    m_buttonBox
        .button_3()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Preset.kShootSpeaker_0m)));

    // ---------------------------------------- BUTTON 4
    //                                          Set Arm to Intake Preset
    m_buttonBox.button_4().onTrue(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup));

    // ---------------------------------------- BUTTON 5
    //                                          Set Arm to Intake Shoot0m
    m_buttonBox
        .button_5()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Preset.kShootInsidePerimeter)));

    // ---------------------------------------- BUTTON 6
    //                                          Set Arm to Stow Preset
    m_buttonBox.button_6().onTrue(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow));

    // ---------------------------------------- BUTTON 7
    //                                          Activate DriveToNote Intaking (Ned's Order)
    m_buttonBox
        .button_7()
        .toggleOnTrue(
            new SequentialCommandGroup(
                    new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup),
                    new IntakeCommand(m_intake)
                        .andThen(new IntakeAdjustmentCommand(m_intake))
                        .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)))
                .deadlineWith(new DriveToNoteCommand(m_drivebase, m_intake)));

    // ---------------------------------------- BUTTON 8
    //                                          Activate DriveToNote Intaking (Documentation Order?)
    m_buttonBox
        .button_8()
        .toggleOnTrue(
            new IntakeCommand(m_intake)
                .deadlineWith(
                    new SequentialCommandGroup(
                        new SetArmCommand(m_armRot, m_armExt, () -> Preset.kFloorPickup),
                        new DriveToNoteCommand(m_drivebase, m_intake)))
                .andThen(new IntakeAdjustmentCommand(m_intake))
                .andThen(new SetArmCommand(m_armRot, m_armExt, () -> Preset.kStow)));

    // ---------------------------------------- BUTTON 9
    //                                          Zero The Arm Extension
    m_buttonBox
        .button_9()
        .onTrue(new InstantCommand(() -> m_armExt.reset())); // Start Zeroing of the arm

    // ---------------------------------------- OTHER TRIGGERS --------------------------
    // ----------------------------------------------------------------------------------
    // ---------------------------------------- Rumble when all subsystems at setpoint
    // Trigger readyToShoot =
    //     new Trigger(
    //         () ->
    //             !m_intake.isEmpty()
    //                 && m_shooter.isAtSpeed()
    //                 && m_armRot.atSetpoint()
    //                 && m_armExt.atSetpoint());
    // m_xboxController.a().and(readyToShoot).onTrue(getAutonomousCommand());
    // readyToShoot.whileTrue(
    //     Commands.startEnd(
    //         () -> {
    //           m_xboxController.getHID().setRumble(RumbleType.kLeftRumble, 0.5);
    //         },
    //         () -> {
    //           m_xboxController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    //         }));

    new Trigger(() -> m_intake.isFull())
        .debounce(0.1)
        .onTrue(new RumbleCommand(m_xboxController.getHID(), 1, 0.5, RumbleType.kBothRumble));
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

    // m_drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation()
    //         ? driveFieldOrientedAnglularVelocity
    //         : driveFieldOrientedDirectAngleSim);

    m_drivebase.setDefaultCommand(
        m_drivebase.driveCommand(
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1) * driveSpeedFactor,
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1) * driveSpeedFactor,
            () -> -m_xboxController.getRightX() * rotationSpeedFactor));

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
        return Commands.print("Print Auto Command");

      case EXIT:
        return new DriveToPoseCommand(
            m_drivebase, new Pose2d(0, 0, Rotation2d.fromDegrees(180)), false);

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
