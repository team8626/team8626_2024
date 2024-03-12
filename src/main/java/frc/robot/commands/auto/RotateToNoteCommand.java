// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.PhotonCamera;

public class RotateToNoteCommand extends Command {

  private final SwerveSubsystem m_drive;
  // P 3.125
  private double m_rotPValue = 3.125;
  private double m_rotIValue = 1.15;
  private double m_rotDValue = 0.2;
  private double m_integratorRangeValue = 5;

  private final ProfiledPIDController m_rotPID =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

  private PhotonCamera m_ODCamera = new PhotonCamera("Arducam_OD003");

  public RotateToNoteCommand(SwerveSubsystem drive) {
    m_drive = drive;

    addRequirements(m_drive);

    // SmartDashboard.putNumber(
    //     "Commands/RotateToNoteCommand/Drive Rotation P Value",
    //     SmartDashboard.getNumber(
    //         "Commands/RotateToNoteCommand/Drive Rotation P Value", m_rotPValue));
    // SmartDashboard.putNumber(
    //     "Commands/RotateToNoteCommand/Drive Rotation I Value",
    //     SmartDashboard.getNumber(
    //         "Commands/RotateToNoteCommand/Drive Rotation I Value", m_rotPValue));
    // SmartDashboard.putNumber(
    //     "Commands/RotateToNoteCommand/Drive Rotation D Value",
    //     SmartDashboard.getNumber(
    //         "Commands/RotateToNoteCommand/Drive Rotation D Value", m_rotPValue));

    // SmartDashboard.putNumber(
    //     "Commands/RotateToNoteCommand/Integrator Range Value",
    //     SmartDashboard.getNumber(
    //         "Commands/RotateToNoteCommand/Integrator Range Value", m_integratorRangeValue));

    setName("RotateToNoteCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LEDSubsystem.setMode((LedMode.FOLLOWNOTE));

    // m_rotPValue =
    //     SmartDashboard.getNumber("Commands/RotateToNoteCommand/Drive Rotation P Value", 0);
    // m_rotIValue =
    //     SmartDashboard.getNumber("Commands/RotateToNoteCommand/Drive Rotation I Value", 0);
    // m_rotDValue =
    //     SmartDashboard.getNumber("Commands/RotateToNoteCommand/Drive Rotation D Value", 0);
    // m_integratorRangeValue =
    //     SmartDashboard.getNumber("Commands/RotateToNoteCommand/Integrator Range Value", 0);

    m_rotPID.setConstraints(
        new TrapezoidProfile.Constraints(
            Constants.Auton.kMaxAngularSpeedRadiansPerSecond * 4,
            Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared * 4));

    m_rotPID.setPID(m_rotPValue, m_rotIValue, m_rotDValue);
    m_rotPID.setIZone(m_integratorRangeValue);

    m_rotPID.setTolerance(Math.toRadians(2), Math.toRadians(2));
    try {
      m_rotPID.reset(Math.toRadians(m_ODCamera.getLatestResult().getBestTarget().getYaw()));
    } catch (NullPointerException e) {
      System.out.println(e);
    }
    ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    try {
      double currentNoteYaw = Math.toRadians(m_ODCamera.getLatestResult().getBestTarget().getYaw());
      // SmartDashboard.putNumber("Commands/RotateToNoteCommand/Yaw of Note", currentNoteYaw);
      m_drive.drive(new ChassisSpeeds(0, 0, m_rotPID.calculate(currentNoteYaw, 0)));
      // SmartDashboard.putNumber("Commands/RotateToNoteCommand/Degree Error",
      // Math.toDegrees(m_rotPID.getPositionError()));
    } catch (NullPointerException e) {
      System.out.println();
    }
    ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDSubsystem.setMode((LedMode.DEFAULT));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_rotPID.atSetpoint();
  }
}
