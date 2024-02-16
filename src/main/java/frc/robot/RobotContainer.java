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
<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
=======
import frc.robot.subsystems.Dashboard;
<<<<<<< HEAD
import frc.robot.subsystems.drive.DriveSubsystem;
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
=======
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.Constants;
=======
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
<<<<<<< HEAD
<<<<<<< HEAD

>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
>>>>>>> 13d8b35 (Update Vendor Libraries to latest version)
import frc.robot.subsystems.swervedrive.Constants.OperatorConstants;
=======
>>>>>>> 616d28e (Import)
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.commands.DriveToPoseCommand;
<<<<<<< HEAD
<<<<<<< HEAD
import frc.robot.subsystems.swervedrive.commands.DriveToPoseTrajPIDCommand;
=======
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
=======
import frc.robot.subsystems.swervedrive.commands.DriveToPoseTrajPIDCommand;
>>>>>>> 58b1481 (DTP Trajectory PID Command Working)
import frc.robot.subsystems.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.utils.CommandButtonController;
<<<<<<< HEAD
import frc.utils.CommandButtonController;
=======
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
import java.io.File;

public class RobotContainer {

<<<<<<< HEAD
<<<<<<< HEAD
  // DriveSubsystem m_drive = new DriveSubsystem();
=======
  DriveSubsystem m_drive = new DriveSubsystem();
=======
  // DriveSubsystem m_drive = new DriveSubsystem();
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
  ShooterSubsystem m_shooter = new ShooterSubsystem();
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
  Dashboard m_dashboard;

  // DriveSubsystem m_drive = new DriveSubsystem();
  public final SwerveSubsystem m_drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public final ArmSubsystem m_arm = new ArmSubsystem();

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final LEDSubsystem m_leds = new LEDSubsystem();

  private final XboxController m_xboxController =
      new XboxController(Constants.OperatorConstants.kXboxControllerPort);
<<<<<<< HEAD

  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.OperatorConstants.kTestControllerPort);

  private final CommandButtonController m_buttonBox =
      new CommandButtonController(Constants.OperatorConstants.kButtonBoxPort);
=======
=======
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
=======
>>>>>>> 5ff9b85 (Pre Debugging (other files))
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final LEDSubsystem m_leds = new LEDSubsystem();

>>>>>>> 70353eb (Dashboard pre work)
  private final XboxController m_xboxController =
      new XboxController(IOControlsConstants.kXboxControllerPort);
=======
>>>>>>> 12f9ff4 (Remove Old Drive Library)

  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.OperatorConstants.kTestControllerPort);

  private final CommandButtonController m_buttonBox =
<<<<<<< HEAD
      new CommandButtonController(IOControlsConstants.kButtonBoxPort);
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
      new CommandButtonController(Constants.OperatorConstants.kButtonBoxPort);
>>>>>>> 12f9ff4 (Remove Old Drive Library)

  public RobotContainer() {

    configureBindings();
    configureDefaultCommands();

<<<<<<< HEAD
<<<<<<< HEAD
    m_dashboard = new Dashboard(m_drivebase, m_arm, m_intake, m_shooter);
=======
    m_dashboard = new Dashboard(m_drivebase, m_arm);
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
    m_dashboard = new Dashboard(m_drivebase, m_arm, m_intake, m_shooter);
>>>>>>> 70353eb (Dashboard pre work)
  }

  private void configureBindings() {

    /** Button Box Bindings */
<<<<<<< HEAD
<<<<<<< HEAD
    // m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(-103)));
    // m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(20.0)));

    // m_buttonBox.button_3().onTrue(new InstantCommand(() -> m_arm.setLeng  thInches(0)));
    // m_buttonBox.button_4().onTrue(new InstantCommand(() -> m_arm.setLengthInches(10)));

<<<<<<< HEAD
    m_buttonBox
        .button_1()
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_intake.start()),
                new InstantCommand(() -> LEDSubsystem.setMode(LedMode.INTAKING))));
    m_buttonBox
        .button_2()
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_intake.stop()),
                new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DEFAULT))));

    m_buttonBox.button_4().onTrue(new InstantCommand(() -> m_shooter.start()));
    m_buttonBox.button_5().onTrue(new InstantCommand(() -> m_shooter.stop()));

    /** Test Controller Buttons * */
    m_buttonBox
        .button_9()
=======
    m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(-103)));
    m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(20.0)));
=======
    // m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(-103)));
    // m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_arm.setAngleDeg(20.0)));
>>>>>>> 70353eb (Dashboard pre work)

    // m_buttonBox.button_3().onTrue(new InstantCommand(() -> m_arm.setLeng  thInches(0)));
    // m_buttonBox.button_4().onTrue(new InstantCommand(() -> m_arm.setLengthInches(10)));

    m_buttonBox.button_1().onTrue(new InstantCommand(() -> m_intake.setMotors(.9)));
    m_buttonBox.button_2().onTrue(new InstantCommand(() -> m_intake.setMotors(0)));

=======
>>>>>>> 5ff9b85 (Pre Debugging (other files))
    m_buttonBox
        .button_1()
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_intake.start()),
                new InstantCommand(() -> LEDSubsystem.setMode(LedMode.INTAKING))));
    m_buttonBox
        .button_2()
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_intake.stop()),
                new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DEFAULT))));

    m_buttonBox.button_4().onTrue(new InstantCommand(() -> m_shooter.start()));
    m_buttonBox.button_5().onTrue(new InstantCommand(() -> m_shooter.stop()));

    /** Test Controller Buttons * */
<<<<<<< HEAD
    m_testController
        .a()
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
    m_buttonBox
        .button_9()
>>>>>>> 70353eb (Dashboard pre work)
        .onTrue(new InstantCommand(() -> m_arm.reset())); // Start Zeroing of the arm
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
    Command controlArm =
        m_arm.controlCommand(
            () -> MathUtil.applyDeadband(m_testController.getLeftY(), 0.1),
            () -> MathUtil.applyDeadband(m_testController.getLeftX(), 0.1));

    m_arm.setDefaultCommand(controlArm);
  }

  public Command getAutonomousCommand() {
    switch (m_dashboard.getSelectedAuto()) {
      default:
        return Commands.print("Print Auto Command");

      case EXIT:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        return new DriveToPoseCommand(
            m_drivebase, new Pose2d(0, 0, Rotation2d.fromDegrees(180)), false);
=======
        return new DriveToPoseCommand(m_drivebase, 0, 0, 90, false);
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
=======
        return new DriveToPoseCommand(m_drivebase, new Pose2d(0, 0, new Rotation2d(90)), false);
>>>>>>> a20f000 (Command Templates)
=======
        return new DriveToPoseCommand(
            m_drivebase, new Pose2d(0, 0, Rotation2d.fromDegrees(180)), false);
>>>>>>> a14a4a7 (Don't use radians as setpoint idiot (Works now))

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
