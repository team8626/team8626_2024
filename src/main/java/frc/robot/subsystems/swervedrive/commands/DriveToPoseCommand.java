// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
import frc.robot.subsystems.swervedrive.Constants;
=======
import frc.robot.subsystems.drive.DriveConstants;
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPoseCommand extends Command {

  private final SwerveSubsystem m_drive;

  private final ProfiledPIDController m_xPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_yPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController m_rotPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private static Pose2d m_pose;

  private double m_xDesiredPos;
  private double m_yDesiredPos;
  private double m_rotDesiredPos;

  // Will only work when atSetpoint() set
  private boolean m_finish;

<<<<<<< HEAD
  public DriveToPoseCommand(SwerveSubsystem drive, Pose2d desiredPose, boolean finish) {
    m_drive = drive;

    m_xDesiredPos = desiredPose.getX();
    m_yDesiredPos = desiredPose.getY();
    m_rotDesiredPos = desiredPose.getRotation().getDegrees();
=======
  public DriveToPoseCommand(
      SwerveSubsystem drive,
      double xDesiredPos,
      double yDesiredPos,
      double rotDesiredPos,
      boolean finish) {
    m_drive = drive;

    m_xDesiredPos = xDesiredPos;
    m_yDesiredPos = yDesiredPos;
    m_rotDesiredPos = rotDesiredPos;
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)

    m_finish = finish;

    addRequirements(m_drive);

<<<<<<< HEAD
<<<<<<< HEAD
    setName("Drive To Pose PID Command");

=======
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
=======
    setName("Drive To Pose PID Command");

>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
    SmartDashboard.putNumber("Drive Position P Value", 0);
    SmartDashboard.putNumber("Drive Position I Value", 0);
    SmartDashboard.putNumber("Drive Position D Value", 0);

    // 0.005
<<<<<<< HEAD
    SmartDashboard.putNumber(
        "Drive Rotation P Value", SmartDashboard.getNumber("Drive Rotation P Value", 0.015));
    SmartDashboard.putNumber("Drive Rotation I Value", 0);
    SmartDashboard.putNumber("Drive Rotation D Value", 0);

    //     SmartDashboard.getNumber("Drive Position P Value", 0);
    //     SmartDashboard.getNumber("Drive Position I Value", 0);
    //     SmartDashboard.getNumber("Drive Position D Value", 0);

    //     // 0.015
    //     SmartDashboard.getNumber("Drive Rotation P Value", 0.01);
    //     SmartDashboard.getNumber("Drive Rotation I Value", 0);
    //    SmartDashboard.getNumber("Drive Rotation D Value", 0);

    SmartDashboard.putNumber("Drive Velocity Constraint", Constants.Auton.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive Acceleration Constraint", Constants.Auton.kMaxAccelerationMetersPerSecondSquared);
    SmartDashboard.putNumber(
        "Rotation Velocity Constraint", Constants.Auton.kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber(
        "Rotation Acceleration Constraint",
        Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared);
=======
    SmartDashboard.putNumber("Drive Rotation P Value", 0.005);
    SmartDashboard.putNumber("Drive Rotation I Value", 0);
    SmartDashboard.putNumber("Drive Rotation D Value", 0);

    SmartDashboard.putNumber(
        "Drive Velocity Constraint", DriveConstants.AutoConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber(
        "Drive Acceleration Constraint",
        DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    SmartDashboard.putNumber(
        "Rotation Velocity Constraint",
        DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber(
        "Rotation Acceleration Constraint",
        DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose = m_drive.getPose();

    double drivePValue = SmartDashboard.getNumber("Drive Position P Value", 0);
    double driveIValue = SmartDashboard.getNumber("Drive Position I Value", 0);
    double driveDValue = SmartDashboard.getNumber("Drive Position D Value", 0);

    // 0.015
<<<<<<< HEAD
    double rotPValue = SmartDashboard.getNumber("Drive Rotation P Value", 0.015);
=======
    double rotPValue = SmartDashboard.getNumber("Drive Rotation P Value", 0.005);
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
    double rotIValue = SmartDashboard.getNumber("Drive Rotation I Value", 0);
    double rotDValue = SmartDashboard.getNumber("Drive Rotation D Value", 0);

    double driveMaxVelocity =
        SmartDashboard.getNumber(
<<<<<<< HEAD
            "Drive Velocity Constraint", Constants.Auton.kMaxSpeedMetersPerSecond);
    double driveMaxAcceleration =
        SmartDashboard.getNumber(
            "Drive Acceleration Constraint",
            Constants.Auton.kMaxAccelerationMetersPerSecondSquared);
    double rotationMaxVelocity =
        SmartDashboard.getNumber(
            "Rotation Velocity Constraint", Constants.Auton.kMaxAngularSpeedRadiansPerSecond);
    double rotationMaxAcceleration =
        SmartDashboard.getNumber(
            "Rotation Acceleration Constraint",
            Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared);
=======
            "Drive Velocity Constraint", DriveConstants.AutoConstants.kMaxSpeedMetersPerSecond);
    double driveMaxAcceleration =
        SmartDashboard.getNumber(
            "Drive Acceleration Constraint",
            DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    double rotationMaxVelocity =
        SmartDashboard.getNumber(
            "Rotation Velocity Constraint",
            DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    double rotationMaxAcceleration =
        SmartDashboard.getNumber(
            "Rotation Acceleration Constraint",
            DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)

    m_xPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    m_rotPID.setConstraints(
        new TrapezoidProfile.Constraints(
            Math.toDegrees(rotationMaxVelocity), Math.toDegrees(rotationMaxAcceleration)));

    m_xPID.setPID(drivePValue, driveIValue, driveDValue);
    m_yPID.setPID(drivePValue, driveIValue, driveDValue);
    m_rotPID.setPID(rotPValue, rotIValue, rotDValue);

    m_xPID.setTolerance(
<<<<<<< HEAD
        Constants.Auton.kDriveXPosSetpointTolerance, Constants.Auton.kDriveXVelSetpointTolerance);
    m_yPID.setTolerance(
        Constants.Auton.kDriveYPosSetpointTolerance, Constants.Auton.kDriveYVelSetpointTolerance);
    m_rotPID.setTolerance(
        Constants.Auton.kDriveRotPosSetpointTolerance,
        Constants.Auton.kDriveRotVelSetpointTolerance);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
    m_rotPID.reset(m_drive.getOdometryHeading().getDegrees());

    m_rotPID.enableContinuousInput(0, 360);
=======
        DriveConstants.Constants.kDriveXPosSetpointTolerance,
        DriveConstants.Constants.kDriveXVelSetpointTolerance);
    m_yPID.setTolerance(
        DriveConstants.Constants.kDriveYPosSetpointTolerance,
        DriveConstants.Constants.kDriveYVelSetpointTolerance);
    m_rotPID.setTolerance(
        DriveConstants.Constants.kDriveRotPosSetpointTolerance,
        DriveConstants.Constants.kDriveRotVelSetpointTolerance);

    m_xPID.reset(m_pose.getX());
    m_yPID.reset(m_pose.getY());
    //  TODO: Re-add this but gyro reading - m_rotPID.reset(m_pose.getRotation().getDegrees());
    m_rotPID.reset(0);

    m_rotPID.enableContinuousInput(-180, 180);
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pose = m_drive.getPose();

    m_drive.drive(
        new ChassisSpeeds(
            m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
            m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
<<<<<<< HEAD
<<<<<<< HEAD
            m_rotPID.calculate(m_drive.getOdometryHeading().getDegrees(), m_rotDesiredPos)));
=======
            m_rotPID.calculate(m_drive.getGyroHeading(), m_rotDesiredPos)));
>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation)
=======
            m_rotPID.calculate(m_drive.getYaw(), m_rotDesiredPos)));
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
    //  SmartDashboard.putNumber("Angle Setpoint", m_rotPID.getPositionError() +
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish && m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
  }
}
