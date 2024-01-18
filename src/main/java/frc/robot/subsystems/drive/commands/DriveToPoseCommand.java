// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveToPoseCommand extends Command {

  private final DriveSubsystem m_drive;

  private final ProfiledPIDController m_xPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(null, null));
  private final ProfiledPIDController m_yPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(null, null));
  private final ProfiledPIDController m_rotPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(null, null));

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_xPID.setConstraints(new TrapezoidProfile.Constraints(0, 0));
    m_yPID.setConstraints(new TrapezoidProfile.Constraints(0, 0));
    m_rotPID.setConstraints(new TrapezoidProfile.Constraints(0, 0));

    m_xPID.setPID(0, 0, 0);
    m_yPID.setPID(0, 0, 0);
    m_rotPID.setPID(0, 0, 0);

    m_xPID.setTolerance(0, 0);	
    m_yPID.setTolerance(0, 0);
    m_rotPID.setTolerance(0, 0);

    m_xPID.reset(0);
    m_yPID.reset(0);
    m_rotPID.reset(0);

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
    return m_xPID.atSetpoint() && m_yPID.atSetpoint() && m_rotPID.atSetpoint();
  }
}
