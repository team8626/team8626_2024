// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TranslateToPositionCommand extends Command {

  private final SwerveSubsystem m_drive;

  private final ProfiledPIDController m_xPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_yPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private static Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;

  // Will only work when atSetpoint() set
  private boolean m_finish;

  public TranslateToPositionCommand(SwerveSubsystem drive, Pose2d desiredPose, boolean finish) {
    m_drive = drive;

    m_xDesiredPos = desiredPose.getX();
    m_yDesiredPos = desiredPose.getY();

    m_finish = finish;

    addRequirements(m_drive);

    setName("Drive To Pose PID Command");

    SmartDashboard.putNumber(
        "Drive Position P Value", SmartDashboard.getNumber("Drive Position P Value", 12.03125));
    SmartDashboard.putNumber(
        "Drive Position I Value", SmartDashboard.getNumber("Drive Position I Value", 0));
    SmartDashboard.putNumber(
        "Drive Position D Value", SmartDashboard.getNumber("Drive Position D Value", 1));

    SmartDashboard.putNumber("Drive Velocity Constraint", Constants.Auton.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive Acceleration Constraint", Constants.Auton.kMaxAccelerationMetersPerSecondSquared);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose = m_drive.getPose();

    double drivePValue = SmartDashboard.getNumber("Drive Position P Value", 0);
    double driveIValue = SmartDashboard.getNumber("Drive Position I Value", 0);
    double driveDValue = SmartDashboard.getNumber("Drive Position D Value", 0);

    double driveMaxVelocity =
        SmartDashboard.getNumber(
            "Drive Velocity Constraint", Constants.Auton.kMaxSpeedMetersPerSecond);
    double driveMaxAcceleration =
        SmartDashboard.getNumber(
            "Drive Acceleration Constraint",
            Constants.Auton.kMaxAccelerationMetersPerSecondSquared);

    m_xPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));

    m_xPID.setPID(drivePValue, driveIValue, driveDValue);
    m_yPID.setPID(drivePValue, driveIValue, driveDValue);

    m_xPID.setTolerance(
        Constants.Auton.kDriveXPosSetpointTolerance, Constants.Auton.kDriveXVelSetpointTolerance);
    m_yPID.setTolerance(
        Constants.Auton.kDriveYPosSetpointTolerance, Constants.Auton.kDriveYVelSetpointTolerance);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pose = m_drive.getPose();

    m_drive.drive(
        new ChassisSpeeds(
            m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
            m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
            0));
    SmartDashboard.putNumber("X Error", m_xPID.getPositionError());
    SmartDashboard.putNumber("Y Error", m_yPID.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish && m_xPID.atSetpoint() && m_yPID.atSetpoint();
  }
}
