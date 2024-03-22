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
import java.util.function.Supplier;

public class TurnToAngleCommand extends Command {

  private final SwerveSubsystem m_drive;

  private final ProfiledPIDController m_rotPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private Supplier<Pose2d> m_desiredPoseSupplier;
  private double rotDesiredPos;
  private double m_angleTolerance;
  private double m_omegaTolerance;

  // Will only work when atSetpoint() set
  private boolean m_finish;

  public TurnToAngleCommand(
      SwerveSubsystem drive,
      Supplier<Pose2d> desiredPoseSupplier,
      double angleTolerance,
      double omegaTolerance,
      boolean finish) {
    m_drive = drive;

    m_desiredPoseSupplier = desiredPoseSupplier;
    m_angleTolerance = angleTolerance;
    m_omegaTolerance = omegaTolerance;

    m_finish = finish;

    addRequirements(m_drive);

    setName("Turn To Angle PID Command");
    // 0.005
    SmartDashboard.putNumber(
        "Drive Rotation P Value", SmartDashboard.getNumber("Drive Rotation P Value", 5));
    SmartDashboard.putNumber(
        "Drive Rotation I Value", SmartDashboard.getNumber("Drive Rotation I Value", 0.7));
    SmartDashboard.putNumber(
        "Drive Rotation D Value", SmartDashboard.getNumber("Drive Rotation D Value", 0.25));
    SmartDashboard.putNumber(
        "Drive Rotation Integrator Zone",
        SmartDashboard.getNumber("Drive Rotation Integrator Zone", 0));

    SmartDashboard.putNumber(
        "Rotation Velocity Constraint", Constants.Auton.kMaxAngularSpeedRadiansPerSecond);
    SmartDashboard.putNumber(
        "Rotation Acceleration Constraint",
        Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotDesiredPos = m_desiredPoseSupplier.get().getRotation().getRadians();

    double rotPValue = SmartDashboard.getNumber("Drive Rotation P Value", 0);
    double rotIValue = SmartDashboard.getNumber("Drive Rotation I Value", 0);
    double rotDValue = SmartDashboard.getNumber("Drive Rotation D Value", 0);
    double rotIZone = SmartDashboard.getNumber("Drive Rotation Integrator Zone", 0);

    double rotationMaxVelocity =
        SmartDashboard.getNumber(
            "Rotation Velocity Constraint", Constants.Auton.kMaxAngularSpeedRadiansPerSecond);
    double rotationMaxAcceleration =
        SmartDashboard.getNumber(
            "Rotation Acceleration Constraint",
            Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared);

    // TODO: Change Unit? If so, increase constraint constants
    m_rotPID.setConstraints(
        new TrapezoidProfile.Constraints(
            Math.toDegrees(rotationMaxVelocity), Math.toDegrees(rotationMaxAcceleration)));

    m_rotPID.setPID(rotPValue, rotIValue, rotDValue);

    // TODO: Implement?
    // Will be able to have a stronger I for steady state error without causing overshoot
    // m_rotPID.setIZone(rotIZone);

    m_rotPID.setTolerance(m_angleTolerance, m_omegaTolerance);

    m_rotPID.reset(m_drive.getOdometryHeading().getRadians());

    m_rotPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drive.drive(
        new ChassisSpeeds(
            0, 0, m_rotPID.calculate(m_drive.getOdometryHeading().getRadians(), rotDesiredPos)));
    SmartDashboard.putNumber("Degree Error", Math.toDegrees(m_rotPID.getPositionError()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish && m_rotPID.atSetpoint();
  }
}
