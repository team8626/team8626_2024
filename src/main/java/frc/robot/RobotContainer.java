// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.DriveToPoseCommand;

public class RobotContainer {

DriveSubsystem m_drive = new DriveSubsystem();

Dashboard m_dashboard;

XboxController m_xboxController = new XboxController(0);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    m_dashboard = new Dashboard(m_drive);

    // TODO: Get rid, only for testing drive to pose
    m_drive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }
  private void configureBindings() {}
  private void configureDefaultCommands() {
          m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_drive.drive(
                MathUtil.applyDeadband(-m_xboxController.getLeftY(), DriveConstants.IOControlsConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_xboxController.getLeftX(), DriveConstants.IOControlsConstants.kDriveDeadband),
                MathUtil.applyDeadband(-m_xboxController.getRightX(), DriveConstants.IOControlsConstants.kDriveDeadband),
                false,
                true),
            m_drive));     
  }
  

  public Command getAutonomousCommand() {
    // return m_drive.getDriveToPoseCommand(1, 1, 45);
    return new SequentialCommandGroup(
      new WaitCommand(0.5),
      new DriveToPoseCommand(m_drive, 0, 0, -90, false));
  }
}
