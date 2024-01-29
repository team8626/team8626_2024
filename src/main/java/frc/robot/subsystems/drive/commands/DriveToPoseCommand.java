// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {

  private final DriveSubsystem m_drive;

  // private final ProfiledPIDController m_xPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  // private final ProfiledPIDController m_yPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
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

    // SmartDashboard.putNumber("Drive Position P Value", 0.75);
    // SmartDashboard.putNumber("Drive Position I Value", 0);
    // SmartDashboard.putNumber("Drive Position D Value", 0);

    // SmartDashboard.putNumber("Drive Rotation P Value", 0);
    // SmartDashboard.putNumber("Drive Rotation I Value", 0);
    // SmartDashboard.putNumber("Drive Rotation D Value", 0);

    SmartDashboard.putNumber("Drive Velocity Constraint", DriveConstants.AutoConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("Drive Acceleration Constraint", DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    SmartDashboard.putNumber("Rotation Velocity Constraint", Math.toDegrees(DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond));
    SmartDashboard.putNumber("Rotation Acceleration Constraint", Math.toDegrees(DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pose = m_drive.getPose();

    double[] drivePIDValues = new double[]{
      SmartDashboard.getNumber("Drive Position P Value", 0), //0.75
      SmartDashboard.getNumber("Drive Position I Value", 0),
      SmartDashboard.getNumber("Drive Position D Value", 0)
    };

    double[] rotPIDValues = new double[]{
      SmartDashboard.getNumber("Drive Rotation P Value", 0),
      SmartDashboard.getNumber("Drive Rotation I Value", 0),
      SmartDashboard.getNumber("Drive Rotation D Value", 0)
    };

    double driveMaxVelocity = SmartDashboard.getNumber("Drive Velocity Constraint", DriveConstants.AutoConstants.kMaxSpeedMetersPerSecond);
    double driveMaxAcceleration = SmartDashboard.getNumber("Drive Acceleration Constraint", DriveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    double rotationMaxVelocity = SmartDashboard.getNumber("Rotation Velocity Constraint", Math.toDegrees(DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond));
    double rotationMaxAcceleration = SmartDashboard.getNumber("Rotation Acceleration Constraint", Math.toDegrees(DriveConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
    
    // m_xPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    // m_yPID.setConstraints(new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    // m_rotPID.setConstraints(new TrapezoidProfile.Constraints(rotationMaxVelocity, rotationMaxAcceleration));
    m_rotPID.setConstraints(new TrapezoidProfile.Constraints(rotationMaxVelocity, rotationMaxAcceleration));

    // m_xPID.setPID(drivePIDValues[0], drivePIDValues[1], drivePIDValues[2]);
    // m_yPID.setPID(drivePIDValues[0], drivePIDValues[1], drivePIDValues[2]);
    m_rotPID.setPID(rotPIDValues[0], rotPIDValues[1], rotPIDValues[2]);

    // m_xPID.setPID(0, 0, 0); //0.75, 0, 0
    // m_yPID.setPID(0, 0, 0);
    // m_rotPID.setPID(0.0015, 0, 0.003);

    // m_xPID.setTolerance(DriveConstants.Constants.kDriveXPosSetpointTolerance, DriveConstants.Constants.kDriveXVelSetpointTolerance);	
    // m_yPID.setTolerance(DriveConstants.Constants.kDriveYPosSetpointTolerance, DriveConstants.Constants.kDriveYVelSetpointTolerance);
    m_rotPID.setTolerance(DriveConstants.Constants.kDriveRotPosSetpointTolerance, DriveConstants.Constants.kDriveRotVelSetpointTolerance);

    // m_xPID.reset(m_pose.getX());
    // m_yPID.reset(m_pose.getY());
    m_rotPID.reset(m_drive.getPose().getRotation().getDegrees());


    m_rotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_pose = m_drive.getPose();

  double m_rotOutput = m_rotPID.calculate(m_drive.getPose().getRotation().getDegrees(), m_rotDesiredPos-180);
  m_drive.drive(0,0,
  // m_xPID.calculate(m_pose.getX(), m_xDesiredPos),
  // m_yPID.calculate(m_pose.getY(), m_yDesiredPos),
   m_rotOutput,
  false,
  false
  );
  SmartDashboard.putNumber("Angle Setpoint", m_rotPID.getSetpoint().position);
  SmartDashboard.putNumber("Angle Output", m_rotOutput);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  m_rotPID.atSetpoint();    
    //return m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();

  }
}
