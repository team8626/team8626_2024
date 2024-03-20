// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

public class AimAndShoot2Command extends Command {

  private SwerveSubsystem m_drive;
  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private ArmRotationSubsystem m_armRot;
  private ArmExtensionSubsystem m_armExt;

  private double m_afterShootDuration = 0.25; // Seconds
  private double m_rotationTimeout = 0; // Seconds: 0 -> No Timeout;

  private double m_rotPID_P = 5;
  private double m_rotPID_I = 0.7;
  private double m_rotPID_D = 0.25;
  private double m_rotMaxVelocityRadPerSec = Constants.Auton.kMaxAngularSpeedRadiansPerSecond;
  private double m_rotMaxAccelerationRadPerSecSqr =
      Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared;
  private double m_rotToleranceAngleDeg = 2;
  private double m_rotToleranceVelocityDegPerSec = 1;
  private Pose2d m_targetPose = new Pose2d();

  private final ProfiledPIDController m_rotPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  ;

  // private Supplier<Pose2d> m_desiredPoseSupplier;
  // private double rotDesiredPos;

  private Timer m_afterShootTimer = new Timer();
  private Timer m_timeoutTimer = new Timer();

  private boolean m_stopShooter;

  public AimAndShoot2Command(
      SwerveSubsystem drive,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt) {

    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_armRot = armRot;
    m_armExt = armExt;

    m_stopShooter = true;

    addRequirements(m_drive, m_intake, m_shooter, m_armRot, m_armExt);
    setName("AimAndShoot2Command");

    SmartDashboard.putNumber(
        "Commands/AimAndShoot2/Drive Rot. P",
        SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. P", m_rotPID_P));
    SmartDashboard.putNumber(
        "Commands/AimAndShoot2/Drive Rot. I",
        SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. I", m_rotPID_I));
    SmartDashboard.putNumber(
        "Commands/AimAndShoot2/Drive Rot. D",
        SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. D", m_rotPID_D));

    SmartDashboard.putNumber(
        "Commands/AimAndShoot2/Drive Rot. Velocity (deg.s-1)",
        Units.radiansToDegrees(m_rotMaxVelocityRadPerSec));
    SmartDashboard.putNumber(
        "Commands/AimAndShoot2/Drive Rot. Acceleration (deg.s-2)",
        Units.radiansToDegrees(m_rotMaxAccelerationRadPerSecSqr));
    SmartDashboard.putNumber("Commands/AimAndShoot2/Degree Error", 0);
  }

  /**
   * Request to not stop shooter Flywheels after shooting.
   *
   * @return the command itself
   */
  public AimAndShoot2Command doNotStopFlyWheels() {
    this.m_stopShooter = false;
    return this;
  }

  /**
   * Set timeout on turn to angle part of the command.
   *
   * @param timeoutSeconds <= 0 means no timeout
   * @return the command itself
   */
  public AimAndShoot2Command withPoseRotationTimeout(double timeoutSeconds) {
    this.m_rotationTimeout = timeoutSeconds;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Read Values from Dashboard
    m_rotPID_P = SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. P", 0);
    m_rotPID_I = SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. I", 0);
    m_rotPID_D = SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. D", 0);
    m_rotMaxVelocityRadPerSec =
        Units.degreesToRadians(
            SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. Velocity (deg.s-1)", 0));
    m_rotMaxAccelerationRadPerSecSqr =
        Units.degreesToRadians(
            SmartDashboard.getNumber("Commands/AimAndShoot2/Drive Rot. Acceleration (deg.s-2)", 0));

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

    System.out.printf(
        "[AimAndShoot2] RPM: (%d/%d), Arm Angle: %f, Arm Ext.: %f, Target Angle: %f\n",
        targetPreset.getTopRPM(),
        targetPreset.getBottomRPM(),
        targetPreset.getRotDegrees(),
        targetPreset.getExtInches(),
        m_targetPose.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Adjust Rotation
    m_drive.drive(
        new ChassisSpeeds(
            0,
            0,
            m_rotPIDController.calculate(
                m_drive.getOdometryHeading().getRadians(),
                m_targetPose.getRotation().getRadians())));

    SmartDashboard.putNumber(
        "Commands/AimAndShoot2/Degree Error",
        Math.toDegrees(m_rotPIDController.getPositionError()));

    // All Systems are at SetPoint, SHOOT!!!
    if (m_armRot.atSetpoint()
        && m_armExt.atSetpoint()
        && m_shooter.isAtSpeed()
        && (m_rotPIDController.atSetpoint()
            || ((m_rotationTimeout > 0) && m_timeoutTimer.hasElapsed(m_rotationTimeout)))) {
      m_intake.start(IntakeConstants.kSpeed_Shoot);
      if (m_timeoutTimer.hasElapsed(m_rotationTimeout)) {
        System.out.printf("[AimAndShoot2] Pose rotation timeout (%f sec)\n", m_rotationTimeout);
      }
      System.out.printf("[AimAndShoot2] All systems at SetPoint - SHOOTING...\n");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.printf("[AimAndShoot2] Ended\n");

    if (m_stopShooter || interrupted) {
      m_shooter.stop();
    }
    m_intake.stop();
    m_timeoutTimer.stop();
    m_afterShootTimer.stop();
    LEDSubsystem.setMode(LedMode.DEFAULT);
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
