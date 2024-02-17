// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;

public class TrackNoteCommand extends Command {

  private final SwerveSubsystem m_drive;

  private XboxController m_XboxController;


  private double m_rotPValue = 0.055;
  private double m_rotIValue = 0;
  private double m_rotDValue = 0;

  private final ProfiledPIDController m_rotPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private PhotonCamera m_ODCamera = new PhotonCamera("Arducam_OD003");

  public TrackNoteCommand(SwerveSubsystem drive, XboxController controller) {
    m_drive = drive;
    m_XboxController = controller;

    addRequirements(m_drive);

    setName("Drive To NOTE PID Command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LEDSubsystem.setMode((LedMode.FOLLOWNOTE));

    SmartDashboard.putNumber("Commands/rotateToNote/Drive Rotation P Value", m_rotPValue);
    SmartDashboard.putNumber("Commands/rotateToNote/Drive Rotation I Value", m_rotIValue);
    SmartDashboard.putNumber("Commands/rotateToNote/Drive Rotation D Value", m_rotDValue);

    double rotationMaxVelocity = Math.PI;
    double rotationMaxAcceleration = Math.PI * 2;

    m_rotPID.setConstraints(
        new TrapezoidProfile.Constraints(
            Math.toDegrees(rotationMaxVelocity), Math.toDegrees(rotationMaxAcceleration)));

    m_rotPID.setPID(m_rotPValue, m_rotIValue, m_rotDValue);

    m_rotPID.setTolerance(0, 0);
    if (m_ODCamera.getLatestResult().hasTargets()) {
      m_rotPID.reset(m_ODCamera.getLatestResult().getBestTarget().getYaw());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_rotPValue = SmartDashboard.getNumber("Commands/driveToNote/Drive Rotation P Value", 0.055);
    m_rotIValue = SmartDashboard.getNumber("Commands/driveToNote/Drive Rotation I Value", 0);
    m_rotDValue = SmartDashboard.getNumber("Commands/driveToNote/Drive Rotation D Value", 0);

    m_rotPID.setPID(m_rotPValue, m_rotIValue, m_rotDValue);

    if (m_ODCamera.getLatestResult().hasTargets()) {
      double currentYaw = m_ODCamera.getLatestResult().getBestTarget().getYaw();
      m_drive.driveCommand(
            () -> MathUtil.applyDeadband(m_XboxController.getLeftX(), 0.1),
            () -> MathUtil.applyDeadband(m_XboxController.getLeftY(), 0.1),
            () -> m_rotPID.calculate(currentYaw, 0));
      //m_drive.drive(new ChassisSpeeds(m_ySpeed, m_xSpeed, m_rotPID.calculate(currentYaw, 0)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDSubsystem.setMode((LedMode.DEFAULT));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_rotPID.atSetpoint() || !m_ODCamera.getLatestResult().hasTargets();
    return false;
  }
}
