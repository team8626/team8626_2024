// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.Preset;
import frc.robot.subsystems.preset.PresetManager;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AimAndDriveCommand extends Command {

  private SwerveSubsystem m_drive;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private ArmRotationSubsystem m_armRot;
  private ArmExtensionSubsystem m_armExt;
  private CommandXboxController m_xboxController;
  private RobotContainer m_robotContainer;

  private double m_afterShootDuration = 0.25; // Seconds

  private double m_rotPID_P = 5;
  private double m_rotPID_I = 0.75;
  private double m_rotPID_D = 0.25;
  private double m_rotMaxVelocityRadPerSec = Constants.Auton.kMaxAngularSpeedRadiansPerSecond;
  private double m_rotMaxAccelerationRadPerSecSqr =
      Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared;
  private double m_rotToleranceAngleDeg = 1;
  private double m_rotToleranceVelocityDegPerSec = 1;
  private Pose2d m_targetPose = new Pose2d();

  private final ProfiledPIDController m_rotPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  ;

  // private Supplier<Pose2d> m_desiredPoseSupplier;
  // private double rotDesiredPos;

  private boolean SHOOT = false;

  private Timer m_afterShootTimer = new Timer();
  private Timer m_timeoutTimer = new Timer();

  private boolean m_stopShooter;

  public AimAndDriveCommand(
      SwerveSubsystem drive,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt,
      CommandXboxController xboxController,
      RobotContainer robotContainer) {

    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_armRot = armRot;
    m_armExt = armExt;
    m_xboxController = xboxController;
    m_robotContainer = robotContainer;

    m_stopShooter = true;

    addRequirements(m_drive, m_intake, m_shooter, m_armRot, m_armExt);
    setName("AimAndDriveCommand");
  }

  /**
   * Request to not stop shooter Flywheels after shooting.
   *
   * @return the command itself
   */
  public AimAndDriveCommand doNotStopFlyWheels() {
    this.m_stopShooter = false;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize Timers
    m_timeoutTimer.reset();
    m_timeoutTimer.start();
    m_afterShootTimer.reset();

    Preset targetPreset = PresetManager.getAimAndShootPreset(m_drive.getPose());
    m_targetPose = targetPreset.getPose();

    // Initiate al Subsystems
    LEDSubsystem.setMode(LedMode.AUTOSHOOT);
    m_shooter.start(targetPreset);
    m_armRot.setAngleDeg(targetPreset.getRotDegrees());
    m_armExt.setLengthInches(targetPreset.getExtInches());

    // Rotation PID Controller
    m_rotPIDController.reset(m_drive.getOdometryHeading().getRadians());
    m_rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotPIDController.setPID(m_rotPID_P, m_rotPID_I, m_rotPID_D);
    m_rotPIDController.setTolerance(
        Units.degreesToRadians(m_rotToleranceAngleDeg),
        Units.degreesToRadians(m_rotToleranceVelocityDegPerSec));

    // TODO: All other values are in radians..., why use degrees? - "But that works!", Ned Finkle
    // m_rotPIDController.setConstraints(
    //     new TrapezoidProfile.Constraints(
    //         m_rotMaxVelocityRadPerSec, m_rotMaxAccelerationRadPerSecSqr));

    m_rotPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            Math.toDegrees(m_rotMaxVelocityRadPerSec),
            Math.toDegrees(m_rotMaxAccelerationRadPerSecSqr)));
    LEDSubsystem.setMode(LedMode.DRIVETOPOSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Preset targetPreset = PresetManager.getAimAndShootPreset(m_drive.getPose());
    m_targetPose = targetPreset.getPose();

    m_shooter.start(targetPreset);
    m_armRot.setAngleDeg(targetPreset.getRotDegrees());
    m_armExt.setLengthInches(targetPreset.getExtInches());

    // Adjust Rotation
    m_drive.drive(
        new Translation2d(
            Math.pow(
                    MathUtil.applyDeadband(
                        -m_xboxController.getLeftY() * m_robotContainer.invert, 0.1),
                    3)
                * 6.36,
            Math.pow(
                    MathUtil.applyDeadband(
                        -m_xboxController.getLeftX() * m_robotContainer.invert, 0.1),
                    3)
                * 6.36),
        m_rotPIDController.calculate(
            m_drive.getOdometryHeading().getRadians(), m_targetPose.getRotation().getRadians()),
        true);

    if (m_xboxController.rightTrigger().getAsBoolean()) {
      SHOOT = true;
    }

    if (SHOOT) {
      m_xboxController.getHID().setRumble(RumbleType.kBothRumble, 1);
      LEDSubsystem.setMode(LedMode.SHOOTING);
    }

    // All Systems are at SetPoint, SHOOT!!!
    if (m_armRot.atSetpoint()
        && m_armExt.atSetpoint()
        && m_shooter.isAtSpeed()
        && m_rotPIDController.atSetpoint()
        && SHOOT) {
      m_intake.start(IntakeConstants.kSpeed_Shoot);
      System.out.printf("[AimAndDrive] All systems at SetPoint - SHOOTING...\n");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.printf("[AimAndDrive] Ended\n");

    if (m_stopShooter || interrupted) {
      m_shooter.stop();
    }
    m_intake.stop();
    m_timeoutTimer.stop();
    m_afterShootTimer.stop();
    LEDSubsystem.setMode(LedMode.DEFAULT);
    m_xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    // Wait to make sure the NOTE finished travelling from Intake to Shooter
    if (m_intake.isEmpty()) {
      m_afterShootTimer.start();
      if (m_afterShootTimer.hasElapsed(m_afterShootDuration) || m_stopShooter) {
        retval = true;
      }
    }

    return retval;
  }
}
