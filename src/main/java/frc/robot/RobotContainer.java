// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Presets.Preset;
import frc.robot.commands.subsystems.drive.DriveToPoseCommand;
import frc.robot.commands.subsystems.drive.DriveToPoseTrajPIDCommand;
import frc.robot.commands.subsystems.intake.IntakeAdjustmentCommand;
import frc.robot.commands.subsystems.intake.IntakeCommand;
import frc.robot.commands.subsystems.shooter.ShooterCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.utils.CommandButtonController;
import java.io.File;

public class RobotContainer {

  int m_angle = 180;
  int m_length = 0;

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
  public final LEDSubsystem m_leds = new LEDSubsystem();

  private final XboxController m_xboxController =
      new XboxController(Constants.OperatorConstants.kXboxControllerPort);

  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.OperatorConstants.kTestControllerPort);

  private final CommandButtonController m_buttonBox =
      new CommandButtonController(Constants.OperatorConstants.kButtonBoxPort);

  public RobotContainer() {

    configureBindings();
    configureDefaultCommands();

    m_dashboard = new Dashboard(m_drivebase, m_armRot, m_armExt, m_intake, m_shooter);
  }

  private void configureBindings() {

    /** Button Box Bindings */
    // m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(-103)));
    // m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(20.0)));

    // m_buttonBox.button_3().onTrue(new InstantCommand(() -> m_arm.setLeng  thInches(0)));
    // m_buttonBox.button_4().onTrue(new InstantCommand(() -> m_arm.setLengthInches(10)));

    // m_buttonBox
    //     .button_1()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> m_intake.start()),
    //             new InstantCommand(() -> LEDSubsystem.setMode(LedMode.INTAKING))));
    // m_buttonBox
    //     .button_2()
    //     .onTrue(
    //         new ParallelCommandGroup(
    //             new InstantCommand(() -> m_intake.stop()),
    //             new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DEFAULT))));

    /** Test Controller Buttons * */
    // m_buttonBox
    //     .button_1()
    //     .toggleOnTrue(new SimpleShooterCommand(Preset.kShootSpeaker_0m, m_shooter));
    // m_buttonBox
    //     .button_2()
    //     .toggleOnTrue(new SimpleShooterCommand(Preset.kShootSpeaker_2m, m_shooter));
    // m_buttonBox
    //     .button_3()
    //     .toggleOnTrue(new SimpleShooterCommand(Preset.kShootSpeaker_3m, m_shooter));

    // m_buttonBox
    //     .button_4()
    //     .toggleOnTrue(
    //         new StartEndCommand(
    //             () -> m_intake.start(IntakeConstants.kSpeed_Intake), () -> m_intake.stop()));
    // m_buttonBox
    //     .button_5()
    //     .toggleOnTrue(
    //         new StartEndCommand(
    //             () -> m_intake.start(IntakeConstants.kSpeed_Adjust), () -> m_intake.stop()));
    // m_buttonBox
    //     .button_6()
    //     .toggleOnTrue(
    //         new StartEndCommand(
    //             () -> m_intake.start(IntakeConstants.kSpeed_Shoot), () -> m_intake.stop()));

    // Set to Angle Testing
    m_testController.b().onTrue(new InstantCommand(() -> m_armRot.setAngleDeg(++m_angle)));
    m_testController.a().onTrue(new InstantCommand(() -> m_armRot.setAngleDeg(--m_angle)));

    m_testController.y().onTrue(new InstantCommand(() -> m_armExt.setLengthInches(++m_length)));
    m_testController.x().onTrue(new InstantCommand(() -> m_armExt.setLengthInches(--m_length)));

    m_buttonBox
        .button_1()
        .onTrue(
            new InstantCommand(() -> m_armRot.setAngleDeg(Preset.kFloorPickup.getRotDegrees())));
    m_buttonBox
        .button_2()
        .onTrue(
            new InstantCommand(
                () -> m_armRot.setAngleDeg(Preset.kShootSpeaker_0m.getRotDegrees())));
    m_buttonBox
        .button_3()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg(Preset.kStow.getRotDegrees())));

    m_buttonBox
        .button_4()
        .onTrue(
            new InstantCommand(() -> m_armExt.setLengthInches(Preset.kFloorPickup.getExtInches())));
    m_buttonBox
        .button_5()
        .onTrue(
            new InstantCommand(
                () -> m_armExt.setLengthInches(Preset.kShootSpeaker_0m.getExtInches())));
    m_buttonBox
        .button_6()
        .onTrue(new InstantCommand(() -> m_armExt.setLengthInches(Preset.kStow.getExtInches())));

    m_buttonBox
        .button_7()
        .toggleOnTrue(new IntakeCommand(m_intake).andThen(new IntakeAdjustmentCommand(m_intake)));

    m_buttonBox
        .button_8()
        .toggleOnTrue(new ShooterCommand(m_intake, m_shooter, Preset.kShootSpeaker_0m));

    m_buttonBox
        .button_9()
        .onTrue(new InstantCommand(() -> m_armExt.reset())); // Start Zeroing of the arm

    // FOR FUTURE USE...

    //         .onlyIf(() -> m_intake.isFull() && !m_intake.limitReached())));

    // m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_shooter.stop()));
    // m_buttonBox.button_5().toggleOnTrue(new ShooterCommand(Shoom_intake, m_shooter, m_leds)

    // m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_arm.getOut()));
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
            m_xboxController::getBButtonPressed,
            m_xboxController::getAButtonPressed,
            m_xboxController::getXButtonPressed,
            m_xboxController::getBButtonPressed);

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
    Command driveFieldOrientedAnglularVelocity =
        m_drivebase.driveCommand(
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
            () -> -m_xboxController.getRawAxis(4));

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
            m_drivebase, new Pose2d(6, 6, Rotation2d.fromDegrees(180)), false);
    }
  }
}
