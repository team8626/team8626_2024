// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AimAndShoot2Command;
import frc.robot.commands.auto.AutoClimbCommand;
import frc.robot.commands.auto.SemiAutoClimbCommand;
import frc.robot.commands.miscellaneous.RumbleCommand;
import frc.robot.commands.presets.ShootFromAmpCommand;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.commands.subsystems.drive.DriveToPoseTrajPIDCommand;
import frc.robot.commands.subsystems.drive.TranslateToPositionCommand;
import frc.robot.commands.subsystems.drive.TurnToAngleCommand;
import frc.robot.commands.subsystems.intake.EjectIntakeCommand;
import frc.robot.commands.subsystems.intake.IntakeAdjustmentCommand;
import frc.robot.commands.subsystems.intake.IntakeCommand;
import frc.robot.commands.subsystems.shooter.ShootAmpCommand;
import frc.robot.commands.subsystems.shooter.SpinAndShootCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDs.LEDConstants.LedAmbienceMode;
import frc.robot.subsystems.LEDs.LEDConstants.LedErrorMode;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionIOReal;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.powermonitor.PowerMonitor;
import frc.robot.subsystems.preset.PresetManager;
import frc.robot.subsystems.preset.Presets;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.utils.AllianceFlipUtil;
import frc.utils.CommandButtonController;
import java.io.File;
import java.util.HashMap;
import java.util.Set;
import java.util.function.Supplier;

public class RobotContainer {

  // DriveSubsystem m_drive = new DriveSubsystem();
  Dashboard m_dashboard;

  // public final ArmSubsystem m_arm = new ArmSubsystem();
  public final ArmRotationSubsystem m_armRot = new ArmRotationSubsystem();
  public final ArmExtensionSubsystem m_armExt = new ArmExtensionSubsystem(new ArmExtensionIOReal());

  // DriveSubsystem m_drive = new DriveSubsystem();
  public final SwerveSubsystem m_drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"), m_armRot, m_armExt);

  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem(new ClimberIOReal());

  public final LEDSubsystem m_leds = new LEDSubsystem();
  public final PresetManager m_presetStorage = new PresetManager();
  public final PowerMonitor m_buckConverterMonitor = new PowerMonitor();

  Supplier<Command> m_presetDTPSupplier =
      //   () ->
      //       new DriveToPoseTrajPIDCommand(m_drivebase, () -> m_presetStorage.get().getPose(),
      // false);
      () -> {
        // Pose2d currentPresetPose = m_presetStorage.get().getPose();
        // Constants.Auton.kDriveRotPosSetpointTolerance
        return new SequentialCommandGroup(
            new TurnToAngleCommand(
                m_drivebase,
                () -> m_presetStorage.get().getPose(),
                Constants.Auton.kDriveRotPosSetpointTolerance + Math.toRadians(2),
                Constants.Auton.kDriveRotVelSetpointTolerance + Math.toRadians(2),
                true),
            new TranslateToPositionCommand(
                m_drivebase, () -> m_presetStorage.get().getPose(), true),
            new TurnToAngleCommand(
                m_drivebase,
                () -> m_presetStorage.get().getPose(),
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true));
      };

  Supplier<Command> m_presetAutoDTPSupplier =
      () -> new DriveToPoseTrajPIDCommand(m_drivebase, m_presetStorage.get().getPose(), false);

  private final CommandXboxController m_xboxController =
      new CommandXboxController(Constants.OperatorConstants.kXboxControllerPort);

  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.OperatorConstants.kTestControllerPort);

  private final CommandButtonController m_buttonBox =
      new CommandButtonController(Constants.OperatorConstants.kButtonBoxPort);

  private boolean isSlowDrive = false;
  private double driveSpeedFactor = 1;
  private double rotationSpeedFactor = 1;

  public final HashMap<String, Command> commandMap = new HashMap<>();
  public final SendableChooser<Command> m_autoChooser;
  public int invert;

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
    configureCommandMap();
    configureBindings();
    configureDefaultCommands();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    m_dashboard =
        new Dashboard(
            m_drivebase,
            m_armRot,
            m_armExt.getIO(),
            m_intake,
            m_shooter,
            m_climber.getIO(),
            m_presetStorage,
            m_buckConverterMonitor);
  }

  private void configureCommandMap() {
    commandMap.put(
        "AutoIntake",
        new SetArmCommand(m_armRot, m_armExt, () -> Presets.kFloorPickup)
            .andThen(new IntakeCommand(m_intake)));
    commandMap.put(
        "AimAndShoot",
        new AimAndShoot2Command(m_drivebase, m_intake, m_shooter, m_armRot, m_armExt)
            .withPoseRotationTimeout(2.0));

    commandMap.put(
        "SetupForSpeaker",
        new SetArmCommand(m_armRot, m_armExt, () -> Presets.kShootSubwoofer)
            .alongWith(new IntakeAdjustmentCommand(m_intake))
            .alongWith(new InstantCommand(() -> m_shooter.start(Presets.kShootSubwoofer))));

    commandMap.put(
        "SetupForStage",
        new SetArmCommand(m_armRot, m_armExt, () -> Presets.kShootStage)
            .alongWith(new IntakeAdjustmentCommand(m_intake))
            .alongWith(new InstantCommand(() -> m_shooter.start(Presets.kShootStage))));

    commandMap.put(
        "PrepareAutoShot",
        new SetArmCommand(m_armRot, m_armExt, () -> Presets.kStow)
            .alongWith(new IntakeAdjustmentCommand(m_intake))
            .alongWith(new InstantCommand(() -> m_shooter.start(Presets.kShootSubwoofer))));
    commandMap.put("Amp", new ShootFromAmpCommand(m_armRot, m_armExt, m_intake, m_shooter));

    commandMap.put(
        "Shooter",
        new SpinAndShootCommand(
            m_intake, m_shooter, m_armRot, m_armExt, () -> Presets.kShootSubwoofer));

    commandMap.put(
        "LightShooter",
        new SpinAndShootCommand(m_intake, m_shooter, m_armRot, m_armExt, () -> Presets.kShootAmp));

    commandMap.put(
        "ShootForSpeaker",
        new SpinAndShootCommand(
                m_intake, m_shooter, m_armRot, m_armExt, () -> Presets.kShootSubwoofer)
            .doNotStopFlyWheels());

    commandMap.put(
        "ShootForStage",
        new SpinAndShootCommand(
            m_intake, m_shooter, m_armRot, m_armExt, () -> Presets.kShootStage));

    NamedCommands.registerCommands(commandMap);
  }

  private void configureBindings() {

    // ---------------------------------------- MAIN CONTROLLER -------------------------
    // ----------------------------------------------------------------------------------
    //

    // ---------------------------------------- Left Bumper
    //                                          Intake (set arm + start intaking))
    m_xboxController
        .leftBumper()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Presets.kFloorPickup)
                .andThen(
                    new IntakeCommand(m_intake, m_armRot, m_armExt)
                        .andThen(new IntakeAdjustmentCommand(m_intake))));

    m_xboxController
        .leftTrigger()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Presets.kSourcePickup)
                .andThen(
                    new IntakeCommand(m_intake)
                        .andThen(new IntakeAdjustmentCommand(m_intake))
                        .andThen(
                            new SetArmCommand(m_armRot, m_armExt, () -> Presets.kStow)
                                .extensionFirst())));

    // ---------------------------------------- Right Bumper
    //                                          Shooting to stored preset settings
    m_xboxController
        .rightBumper()
        .and(() -> m_presetStorage.get() != Presets.kShootAmp)
        .toggleOnTrue(
            new SpinAndShootCommand(
                    m_intake, m_shooter, m_armRot, m_armExt, () -> m_presetStorage.get())
                .andThen(
                    new SetArmCommand(m_armRot, m_armExt, () -> Presets.kStow).extensionFirst()));

    m_xboxController
        .rightBumper()
        .and(() -> m_presetStorage.get() == Presets.kShootAmp)
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Presets.kShootAmp)
                .andThen(new ShootAmpCommand(m_intake, m_shooter))
                .andThen(
                    new SetArmCommand(m_armRot, m_armExt, () -> Presets.kStow).extensionFirst()));

    // ---------------------------------------- Right Trigger
    //                                          Aim and Shoot
    m_xboxController
        .rightTrigger()
        .toggleOnTrue(
            new AimAndShoot2Command(m_drivebase, m_intake, m_shooter, m_armRot, m_armExt)
                .withPoseRotationTimeout(2)
                .handleInterrupt(
                    () ->
                        new SetArmCommand(m_armRot, m_armExt, () -> Presets.kStow)
                            .extensionFirst()));

    // ---------------------------------------- POV
    //                                          Robot angle
    m_xboxController
        .povUp()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () -> new Pose2d(new Translation2d(), new Rotation2d(0)),
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true));
    m_xboxController
        .povLeft()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () -> new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true));
    m_xboxController
        .povDown()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () -> new Pose2d(new Translation2d(), new Rotation2d(Math.PI)),
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true));
    m_xboxController
        .povRight()
        .onTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () -> new Pose2d(new Translation2d(), new Rotation2d(-Math.PI / 2)),
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true));

    // ---------------------------------------- X
    //                                          Drive to Pose

    // m_xboxController.x().toggleOnTrue(new DeferredCommand(m_presetDTPSupplier, Set.of()));
    m_xboxController
        .x()
        .toggleOnTrue(
            new SequentialCommandGroup(
                new TurnToAngleCommand(
                    m_drivebase,
                    () -> m_presetStorage.get().getPose(),
                    Constants.Auton.kDriveRotPosSetpointTolerance + Math.toRadians(3),
                    Constants.Auton.kDriveRotVelSetpointTolerance + Math.toRadians(2),
                    true),
                new TranslateToPositionCommand(
                    m_drivebase, () -> m_presetStorage.get().getPose(), true),
                new TurnToAngleCommand(
                    m_drivebase,
                    () -> m_presetStorage.get().getPose(),
                    Constants.Auton.kDriveRotPosSetpointTolerance,
                    Constants.Auton.kDriveRotVelSetpointTolerance,
                    true)));

    // ---------------------------------------- Y
    //                                          Eject
    m_xboxController.y().toggleOnTrue(new EjectIntakeCommand(m_intake));

    // ---------------------------------------- A
    //                                          Toggle Slow
    m_xboxController.a().onTrue(new InstantCommand(() -> toggleSlowDrive()));

    // ---------------------------------------- B
    //                                          Turn to Source Angle
    m_xboxController
        .b()
        .toggleOnTrue(
            new TurnToAngleCommand(
                m_drivebase,
                () ->
                    AllianceFlipUtil.apply(
                        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-60)))),
                Units.degreesToRadians(2),
                Units.degreesToRadians(2),
                true));

    // ---------------------------------------- TEST CONTROLLER -------------------------
    // ----------------------------------------------------------------------------------
    //
    // ---------------------------------------- Set to Angle Testing (Test Controller)
    //                                          A/X Rotation
    //                                          X/B Extension
    m_testController
        .y()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg((m_armRot.getRotDegrees() - 2.5))));
    m_testController
        .a()
        .onTrue(new InstantCommand(() -> m_armRot.setAngleDeg((m_armRot.getRotDegrees() + 2.5))));
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
                () -> m_shooter.start(m_presetStorage.get()), () -> m_shooter.stop()));

    // ---------------------------------------- Climb
    m_testController
        .leftTrigger()
        .whileTrue(new InstantCommand(() -> m_climber.setPower(-1.0)))
        .onFalse(new InstantCommand(() -> m_climber.setPower(0)));

    m_testController
        .rightTrigger()
        .whileTrue(new InstantCommand(() -> m_climber.setPower(1.0)))
        .onFalse(new InstantCommand(() -> m_climber.setPower(0)));

    m_testController
        .povUp()
        .toggleOnTrue(
            new SetArmCommand(m_armRot, m_armExt, () -> Presets.kClimbPreset)
                .alongWith(
                    new InstantCommand(
                        () -> LEDSubsystem.setAmbienceMode(LedAmbienceMode.OFF), m_leds)));

    m_testController
        .povRight()
        .onTrue(
            new SemiAutoClimbCommand(m_armRot, m_armExt, m_climber)
                .alongWith(
                    new InstantCommand(
                        () -> LEDSubsystem.setAmbienceMode(LedAmbienceMode.RAINBOW), m_leds)));

    m_testController
        .povDown()
        .onTrue(new SetArmCommand(m_armRot, m_armExt, () -> Presets.kClimbEnd));

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

    // ---------------------------------------- BUTTON 1
    //                                          Preset: Amp
    m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_presetStorage.set(Presets.kShootAmp)));

    // ---------------------------------------- BUTTON 2
    //                                          Preset:  Subwoofer
    m_buttonBox
        .button_2()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Presets.kShootSubwoofer)));

    // ---------------------------------------- BUTTON 3
    //                                          Preset:  Podium
    m_buttonBox
        .button_3()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Presets.kShootPodium)));

    // ---------------------------------------- BUTTON 4
    //                                          Request for Amplification to Human Player
    // m_buttonBox.button_4().toggleOnTrue(m_leds.setModeCommand(LedMode.AMPLIFICATION));
    m_buttonBox.button_4().toggleOnTrue(m_leds.setErrorModeCommand(LedErrorMode.ERROR_GIMME_LIGHT));

    // ---------------------------------------- BUTTON 5
    //                                          Preset: Under Stage
    m_buttonBox
        .button_5()
        .onTrue(new InstantCommand(() -> m_presetStorage.set(Presets.kShootStage)));

    // ---------------------------------------- BUTTON 6
    //                                          Preset: Long Pass
    m_buttonBox.button_6().onTrue(new InstantCommand(() -> m_presetStorage.set(Presets.kLongPass)));

    // ---------------------------------------- BUTTON 7
    //                                          Set Arm to Stow Preset
    m_buttonBox
        .button_7()
        .onTrue(new SetArmCommand(m_armRot, m_armExt, () -> Presets.kStow).extensionFirst());

    // ---------------------------------------- BUTTON 8
    //                                          Climb
    m_buttonBox
        .button_8()
        .toggleOnTrue(
            new AutoClimbCommand(
                    m_drivebase,
                    m_armRot,
                    m_armExt,
                    m_climber,
                    () -> PresetManager.getClosestClimbingStart(m_drivebase.getPose()))
                .alongWith(
                    new InstantCommand(
                        () -> LEDSubsystem.setAmbienceMode(LedAmbienceMode.RAINBOW), m_leds))
                .handleInterrupt(() -> LEDSubsystem.setAmbienceMode(LedAmbienceMode.OFF))
                .handleInterrupt(() -> LEDSubsystem.setMode(LedMode.DEFAULT)));

    // ---------------------------------------- BUTTON 9
    //                                          Zero The Arm Extension
    m_buttonBox
        .button_9()
        .onTrue(new InstantCommand(() -> m_armExt.reset())); // Start Zeroing of the arm

    // ---------------------------------------- OTHER TRIGGERS --------------------------
    // ----------------------------------------------------------------------------------
    // ---------------------------------------- Rumble when all subsystems at setpoint

    new Trigger(() -> m_intake.isFull())
        .debounce(0.1)
        .onTrue(new RumbleCommand(m_xboxController.getHID(), 1, 0.5, RumbleType.kBothRumble));

    // Shooter Auto Spin Trigger
    Trigger autoSpinRadiusTrigger =
        new Trigger(
            () -> {
              Pose2d currentPose = m_drivebase.getPose();

              return (Math.abs(Presets.kShootSubwoofer.getPose().getY() - currentPose.getY())
                  < 4.0); // 4.0m to the speaker
            });

    autoSpinRadiusTrigger.and(() -> m_intake.isFull()).onTrue(m_shooter.setRPMCommand(3000, 3500));
    autoSpinRadiusTrigger.onFalse(new InstantCommand(() -> m_shooter.stop()));
  }

  private void configureDefaultCommands() {
    // AbsoluteDriveAdv closedAbsoluteDriveAdv =
    //     new AbsoluteDriveAdv(
    //         m_drivebase,
    //         () ->
    //             MathUtil.applyDeadband(
    //                 -m_xboxController.getLeftY() * invert, OperatorConstants.LEFT_Y_DEADBAND),
    //         () ->
    //             MathUtil.applyDeadband(
    //                 m_xboxController.getLeftX() * invert, OperatorConstants.LEFT_X_DEADBAND),
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
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY() * invert, 0.1),
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX() * invert, 0.1),
            () -> m_xboxController.getRightX(),
            () -> m_xboxController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // driveFieldOrientedAnglularVelocity =
    //     m_drivebase.driveCommand(
    //         () ->
    //             MathUtil.applyDeadband(-m_xboxController.getLeftY() * invert, 0.1)
    //                 * driveSpeedFactor,
    //         () ->
    //             MathUtil.applyDeadband(-m_xboxController.getLeftX() * invert, 0.1)
    //                 * driveSpeedFactor,
    //         () -> -m_xboxController.getRawAxis(4) * rotationSpeedFactor);

    // driveFieldOrientedAnglularVelocity.setName("Drive Field Oriented Anglular Velocity Command");

    Command driveFieldOrientedDirectAngleSim =
        m_drivebase.simDriveCommand(
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftY() * invert, 0.1),
            () -> MathUtil.applyDeadband(-m_xboxController.getLeftX() * invert, 0.1),
            () -> -m_xboxController.getRawAxis(4));

    Command driveFieldOrientedAnglularVelocity =
        m_drivebase.driveCommand(
            () ->
                MathUtil.applyDeadband(-m_xboxController.getLeftY() * invert, 0.1)
                    * driveSpeedFactor,
            () ->
                MathUtil.applyDeadband(-m_xboxController.getLeftX() * invert, 0.1)
                    * driveSpeedFactor,
            () -> -m_xboxController.getRightX() * rotationSpeedFactor);

    m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation()
            ? driveFieldOrientedAnglularVelocity
            : driveFieldOrientedAnglularVelocity);

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
        return new SpinAndShootCommand(
            m_intake, m_shooter, m_armRot, m_armExt, () -> Presets.kShootSubwoofer);

      case PRINT:
        return Commands.print("Print Auto Command");

      case DO_NOTHING:
        return Commands.none();

      case TRAJECTORY_DTP:
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_presetStorage.set(Presets.kShootSubwoofer)),
            new DeferredCommand(m_presetAutoDTPSupplier, Set.of()),
            new InstantCommand(() -> m_presetStorage.set(Presets.kShootAmp)),
            new DeferredCommand(m_presetAutoDTPSupplier, Set.of()));

      case SHOOT_IN_PLACE:
        return new SpinAndShootCommand(
            m_intake, m_shooter, m_armRot, m_armExt, () -> Presets.kShootSubwoofer);
      case TRAJECTORY:
        return m_autoChooser.getSelected();
    }
  }
}
