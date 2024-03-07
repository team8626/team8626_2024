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

  private double m_robotX = 0;
  private double m_robotY = 0;
  private double g = 9.81; // gravity

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

    m_targetHeight = Units.inchesToMeters(70);

    m_targetDistance =
        Math.sqrt(
            (m_targetX - m_robotX) * (m_targetX - m_robotX)
                + (m_targetY - m_robotY) * (m_targetY - m_robotY));
    m_targetElevation =
        new Rotation2d(Math.atan((FieldConstants.topRightSpeaker.getZ() / m_targetDistance)));

    m_targetRotation =
        AllianceFlipUtil.apply(
            new Rotation2d(Math.acos((m_targetY - m_robotY) / m_targetDistance) - Math.PI / 2));

    m_launchAngle = m_targetElevation;
    double x = m_targetDistance;
    double y = FieldConstants.topRightSpeaker.getZ();
    double o = m_launchAngle.getRadians();

    // v = (sqrt(g) * sqrt(x) * sqrt((tan(o)*tan(o))+1)) / sqrt(2 * tan(o) - (2 * g * y) / x)
    // m_launchVelocity =
    //     (Math.sqrt(g) * Math.sqrt(x) * Math.sqrt((Math.tan(o) * Math.tan(o)) + 1))
    //         / Math.sqrt(2 * Math.tan(o) - (2 * g * y) / x); // velocity
    m_launchVelocity = Math.sqrt(y * 2 * g) / m_targetElevation.getSin();

    SmartDashboard.putNumber("Commands/AimAndShootCommand/Target Distance (m)", m_targetDistance);
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Target Rotation (deg)", m_targetRotation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Target Elevation (deg)", m_targetElevation.getDegrees());
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Launch Angle (deg)", m_launchAngle.getDegrees());
    SmartDashboard.putNumber(
        "Commands/AimAndShootCommand/Launch Velocity (m.s-1)", m_launchVelocity);
    SmartDashboard.putNumber("Commands/AimAndShootCommand/Launch Speed (RPM)", 0);
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
