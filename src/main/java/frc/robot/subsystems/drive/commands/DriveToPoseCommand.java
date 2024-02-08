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

    SmartDashboard.putNumber("Drive Position P Value", 1);
    SmartDashboard.putNumber("Drive Position I Value", 0);
    SmartDashboard.putNumber("Drive Position D Value", 0);

    SmartDashboard.putNumber("Drive Rotation P Value", 0.005);
    SmartDashboard.putNumber("Drive Rotation I Value", 0);
    SmartDashboard.putNumber("Drive Rotation D Value", 0);
    

    SmartDashboard.putNumber("Drive Velocity Constraint", DriveConstants.AutoConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Drive Acceleration Constraint", DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    SmartDashboard.putNumber("Rotation Velocity Constraint", DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber("Rotation Acceleration Constraint", DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose = m_drive.getPose();

    double drivePValue = SmartDashboard.getNumber("Drive Position P Value", 1);
    double driveIValue = SmartDashboard.getNumber("Drive Position I Value", 0);
    double driveDValue = SmartDashboard.getNumber("Drive Position D Value", 0);

    // 0.015
    double rotPValue = SmartDashboard.getNumber("Drive Rotation P Value", 0.015);
    double rotIValue = SmartDashboard.getNumber("Drive Rotation I Value", 0);
    double rotDValue = SmartDashboard.getNumber("Drive Rotation D Value", 0);


    double driveMaxVelocity = SmartDashboard.getNumber("Drive Velocity Constraint", DriveConstants.AutoConstants.kMaxSpeedMetersPerSecond);
    double driveMaxAcceleration = SmartDashboard.getNumber("Drive Acceleration Constraint", DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    double rotationMaxVelocity = SmartDashboard.getNumber("Rotation Velocity Constraint", DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    double rotationMaxAcceleration = SmartDashboard.getNumber("Rotation Acceleration Constraint", DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    
    m_xPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_rotPID.setConstraints(new TrapezoidProfile.Constraints(Math.toDegrees(rotationMaxVelocity), Math.toDegrees(rotationMaxAcceleration)));

    m_xPID.setPID(drivePValue, driveIValue, driveDValue);
    m_yPID.setPID(drivePValue, driveIValue, driveDValue);
    m_rotPID.setPID(rotPValue, rotIValue, rotDValue);

    m_xPID.setTolerance(DriveConstants.Constants.kDriveXPosSetpointTolerance, DriveConstants.Constants.kDriveXVelSetpointTolerance);	
    m_yPID.setTolerance(DriveConstants.Constants.kDriveYPosSetpointTolerance, DriveConstants.Constants.kDriveYVelSetpointTolerance);
    m_rotPID.setTolerance(DriveConstants.Constants.kDriveRotPosSetpointTolerance, DriveConstants.Constants.kDriveRotVelSetpointTolerance);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
    // m_rotPID.reset(m_pose.getRotation().getDegrees());
    m_rotPID.reset(0);

    m_rotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_pose = m_drive.getPose();

  m_drive.drive(
  /* m_xPID.calculate(m_pose.getX(), m_xDesiredPos) */0,
  /* m_yPID.calculate(m_pose.getY(), m_yDesiredPos) */0,
  m_rotPID.calculate(m_drive.getHeading(), m_rotDesiredPos),
  false,
  false
  );
 SmartDashboard.putNumber("Angle Setpoint", m_rotPID.getPositionError() + m_drive.getHeading());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
  }
}