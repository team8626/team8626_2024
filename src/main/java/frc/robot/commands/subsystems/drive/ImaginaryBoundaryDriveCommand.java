// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.ImaginaryBoundary;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

// Drive command that lets the robot drive within rectangular bounds; requires unique april tag
// setup TODO: CLEMENT
public class ImaginaryBoundaryDriveCommand extends Command {
  private SwerveSubsystem m_drive;
  private ImaginaryBoundary m_boundary;
  private DoubleSupplier m_xInput;
  private DoubleSupplier m_yInput;
  private DoubleSupplier m_rotInput;

  private static enum BoundaryStates {
    WITHIN_BOUNDARY,
    WITHIN_ACTIVATION_ZONE,
    STOPPED_WITHIN_ACTIVATION_ZONE
  }

  private static PIDController xSlowPID = new PIDController(0, 0, 0);
  private static PIDController ySlowPID = new PIDController(0, 0, 0);
  private static PIDController xStopPID = new PIDController(0, 0, 0);
  private static PIDController yStopPID = new PIDController(0, 0, 0);
  private static double atRestVelTolerance = 0.2;
  private static double atRestAccTolerance = 0.1;

  private Pose2d m_currentPose;
  private BoundaryStates m_currentState;
  private double m_centerX;
  private double m_centerY;

  public ImaginaryBoundaryDriveCommand(
      SwerveSubsystem drive,
      ImaginaryBoundary boundary,
      DoubleSupplier xInput,
      DoubleSupplier yInput,
      DoubleSupplier rotInput) {
    m_boundary = boundary;
    m_drive = drive;
    m_xInput = xInput;
    m_yInput = yInput;
    m_rotInput = rotInput;
    m_centerX = boundary.m_width / 2;
    m_centerY = boundary.m_height / 2;

    xSlowPID.reset();
    ySlowPID.reset();
    xStopPID.reset();
    yStopPID.reset();

    xSlowPID.setSetpoint(m_centerX);
    ySlowPID.setSetpoint(m_centerY);
    xStopPID.setSetpoint(0);
    yStopPID.setSetpoint(0);

    xStopPID.setTolerance(atRestVelTolerance, atRestAccTolerance);
    yStopPID.setTolerance(atRestVelTolerance, atRestAccTolerance);

    addRequirements(drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPose = m_drive.getPose();
    double xPos = m_currentPose.getX();
    double yPos = m_currentPose.getY();

    if ((xPos > (m_boundary.m_activationZone)
            && xPos < (m_boundary.m_width - m_boundary.m_activationZone))
        && (yPos > (m_boundary.m_activationZone)
            && yPos < (m_boundary.m_height - m_boundary.m_activationZone))) {
      m_currentState = BoundaryStates.WITHIN_BOUNDARY;
    } else {
      if (m_currentState != BoundaryStates.STOPPED_WITHIN_ACTIVATION_ZONE) {
        m_currentState = BoundaryStates.WITHIN_ACTIVATION_ZONE;
      }
    }

    // TODO: Use different drive function? Field relative?
    switch (m_currentState) {
      case WITHIN_BOUNDARY:
        double xSlowOutput = m_xInput.getAsDouble() - xSlowPID.calculate(xPos);
        double ySlowOutput = m_yInput.getAsDouble() - ySlowPID.calculate(yPos);
        double rotSlowOutput = m_rotInput.getAsDouble();

        m_drive.drive(new ChassisSpeeds(xSlowOutput, ySlowOutput, rotSlowOutput));
        break;

      case WITHIN_ACTIVATION_ZONE:
        ChassisSpeeds vel = m_drive.getFieldVelocity();
        double xStopOutput = xStopPID.calculate(vel.vxMetersPerSecond);
        double yStopOutput = yStopPID.calculate(vel.vyMetersPerSecond);
        double rotStopOutput = m_rotInput.getAsDouble();
        m_drive.drive(new ChassisSpeeds(xStopOutput, yStopOutput, rotStopOutput));

        if (xStopPID.atSetpoint() && yStopPID.atSetpoint())
          m_currentState = BoundaryStates.STOPPED_WITHIN_ACTIVATION_ZONE;
        break;

      case STOPPED_WITHIN_ACTIVATION_ZONE:
        boolean xInputInwards = ((m_centerX - xPos) * m_xInput.getAsDouble()) > 0;
        boolean yInputInwards = ((m_centerY - yPos) * m_yInput.getAsDouble()) > 0;
        double xStoppedOutput = xInputInwards ? m_xInput.getAsDouble() : 0;
        double yStoppedOutput = yInputInwards ? m_yInput.getAsDouble() : 0;
        double rotStoppedOutput = m_rotInput.getAsDouble();
        m_drive.drive(new ChassisSpeeds(xStoppedOutput, yStoppedOutput, rotStoppedOutput));
        break;
    }
  }
}
