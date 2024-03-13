// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.utils.AllianceFlipUtil;

public class AimAndShootCommand extends Command {

  private final SwerveSubsystem m_drive;
  private double m_targetX = 0;
  private double m_targetY = 0;
  private double m_targetDistance = 0;
  private double m_targetHeight = 0;

  private Rotation2d m_targetRotation;
  private Rotation2d m_targetElevation;
  private Rotation2d m_launchAngle;

  private double m_launchVelocity = 0;
  private double m_launchRPM = 0;
  private double m_vX = 0;
  private double m_vZ = 0;
  private double m_ooomf = 0;
  private double m_tm = 0;

  private double m_robotX = 0;
  private double m_robotY = 0;
  private double g = 9.81; // gravity

  private double m_z0 = Units.inchesToMeters(20); // Shooting Height

  public AimAndShootCommand(SwerveSubsystem drive) {
    m_drive = drive;

    addRequirements(m_drive);

    setName("Aim and Shoot Command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LEDSubsystem.setMode((LedMode.FOLLOWNOTE));

    m_targetX = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getX();
    m_targetY = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getY();

    m_robotX = m_drive.getPose().getX();
    m_robotY = m_drive.getPose().getY();

    m_targetHeight = Units.inchesToMeters(80.5);

    // TODO: For Testing puspose only
    m_robotX = Units.feetToMeters(9);
    m_robotY = m_targetY;

    m_targetDistance =
        Math.sqrt(
            (m_targetX - m_robotX) * (m_targetX - m_robotX)
                + (m_targetY - m_robotY) * (m_targetY - m_robotY));

    m_targetElevation =
        new Rotation2d(Math.atan((FieldConstants.topRightSpeaker.getZ() / m_targetDistance)));

    m_targetRotation =
        AllianceFlipUtil.apply(
            new Rotation2d(Math.acos((m_targetY - m_robotY) / m_targetDistance) - Math.PI / 2));

    m_ooomf = 0;
    m_vZ = Math.sqrt((m_ooomf * m_ooomf) + (m_targetHeight - m_z0) * 2 * g);
    m_tm = (m_vZ - m_ooomf) / g;
    m_vX = m_targetDistance / m_tm;

    m_launchAngle = new Rotation2d(Math.atan(m_vZ / m_vX));
    m_launchVelocity = Math.sqrt((m_vX * m_vX) + (m_vZ * m_vZ));
    m_launchRPM =
        (m_launchVelocity / (Math.PI * ShooterConstants.kFlywheelDiameterMeters / 2)) * 60;

    //  ((m_launchVelocity) / (Math.PI * (ShooterConstants.kFlywheelDiameterMeters / 2))) * 60;

    SmartDashboard.putNumber("Commands/AimAndShootCommand/Robot X", m_robotX);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Robot Y", m_robotY);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Shooter Z (z0)", m_z0);

    SmartDashboard.putNumber("Commands/AimAndShootCommand/Target X", m_targetX);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Target Y", m_targetY);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Target Z", m_targetHeight);

    SmartDashboard.putNumber("Commands/AimAndShootCommand/v0", m_targetHeight);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Target Z", m_targetHeight);

    SmartDashboard.putNumber("Commands/AimAndShootCommand/Target Distance (m)", m_targetDistance);

    SmartDashboard.putNumber("Commands/AimAndShootCommand/vX", m_vX);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/vZ", m_vZ);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/tM", m_tm);

    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Target Rotation (deg)", m_targetRotation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Target Elevation (deg)", m_targetElevation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Launch Angle (deg)", m_launchAngle.getDegrees());
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Launch Velocity (m.s-1)", m_launchVelocity);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Launch Speed (RPM)", m_launchRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDSubsystem.setMode((LedMode.DEFAULT));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
