// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {

  private final DriveSubsystem m_drive;

  private final ProfiledPIDController m_xPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_yPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_rotPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private static Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;
  private double m_rotDesiredPos;

  public DriveToPoseCommand(DriveSubsystem drive, double xDesiredPos, double yDesiredPos, double rotDesiredPos) {
    m_drive = drive;
    m_xDesiredPos = xDesiredPos;
    m_yDesiredPos = yDesiredPos;
    m_rotDesiredPos = rotDesiredPos;

    addRequirements(m_drive);

    SmartDashboard.putNumberArray("Drive Position PID Values", new double[]{0, 0 ,0});
    SmartDashboard.putNumberArray("Drive Rotation PID Values", new double[]{0, 0 ,0});

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose = m_drive.getPose();

    double[] drivePIDValues = SmartDashboard.getNumberArray("Drive Position PID Values", new double[]{0, 0, 0});
    double[] rotPIDValues = SmartDashboard.getNumberArray("Drive Rotation PID Values", new double[]{0, 0, 0});

    m_xPID.setConstraints(new TrapezoidProfile.Constraints(DriveConstants.Constants.kDriveMaxVelocity * DriveConstants.Constants.kDriveConstraintFactor, DriveConstants.Constants.kDriveMaxAcceleration * DriveConstants.Constants.kDriveConstraintFactor));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(DriveConstants.Constants.kDriveMaxVelocity * DriveConstants.Constants.kDriveConstraintFactor, DriveConstants.Constants.kDriveMaxAcceleration * DriveConstants.Constants.kDriveConstraintFactor));
    m_rotPID.setConstraints(new TrapezoidProfile.Constraints(DriveConstants.Constants.kRotationMaxVelocity * DriveConstants.Constants.kRotateConstraintFactor, DriveConstants.Constants.kRotationMaxVelocity * DriveConstants.Constants.kRotateConstraintFactor));

    m_xPID.setPID(drivePIDValues[0], drivePIDValues[1], drivePIDValues[2]);
    m_yPID.setPID(drivePIDValues[0], drivePIDValues[1], drivePIDValues[2]);
    m_rotPID.setPID(rotPIDValues[0], rotPIDValues[1], rotPIDValues[2]);

    m_xPID.setTolerance(DriveConstants.Constants.kDriveXPosSetpointTolerance, DriveConstants.Constants.kDriveXVelSetpointTolerance);	
    m_yPID.setTolerance(DriveConstants.Constants.kDriveYPosSetpointTolerance, DriveConstants.Constants.kDriveYVelSetpointTolerance);
    m_rotPID.setTolerance(DriveConstants.Constants.kDriveRotPosSetpointTolerance, DriveConstants.Constants.kDriveRotVelSetpointTolerance);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
    m_rotPID.reset(m_pose.getRotation().getDegrees());

    m_rotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_pose = m_drive.getPose();

  m_drive.drive(
  m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
  m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
  /*  m_rotPID.calculate(m_pose.getRotation().getDegrees(), m_rotDesiredPos */ 0,
  false,
  false
  );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xPID.atSetpoint() && m_yPID.atSetpoint()/* && m_rotPID.atSetpoint() */;
  }
}
