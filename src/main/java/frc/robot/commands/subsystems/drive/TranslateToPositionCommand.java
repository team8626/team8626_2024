// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TranslateToPositionCommand extends Command {

  private final SwerveSubsystem m_drive;

  private final double kPValue = 12.03125;
  private final double kIValue = 0;
  private final double kDValue = 1;

  private final ProfiledPIDController m_xPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private final ProfiledPIDController m_yPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

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

    setName("Translate to Position Command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double driveMaxVelocity = Constants.Auton.kMaxSpeedMetersPerSecond;
    double driveMaxAcceleration = Constants.Auton.kMaxAccelerationMetersPerSecondSquared;

    m_xPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));

    m_xPID.setPID(kPValue, kIValue, kDValue);
    m_yPID.setPID(kPValue, kIValue, kDValue);

    m_xPID.setTolerance(1, 0.5);
    m_yPID.setTolerance(1, 0.5);

    m_xPID.reset(m_drive.getPose().getX());
    m_yPID.reset(m_drive.getPose().getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_drive.getPose();

    m_drive.drive(
        new ChassisSpeeds(
            m_xPID.calculate(currentPose.getX()), m_xPID.calculate(currentPose.getY()), 0));
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
