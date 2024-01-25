// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {

  DriveSubsystem m_drive = new DriveSubsystem();

  Dashboard m_dashboard;

  XboxController m_xboxController = new XboxController(0);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();

    m_dashboard = new Dashboard(null);
  }

  private void configureBindings() {}

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_drive.drive(
                    MathUtil.applyDeadband(
                        -m_xboxController.getLeftY(),
                        DriveConstants.IOControlsConstants.kDriveDeadband),
                    MathUtil.applyDeadband(
                        -m_xboxController.getLeftX(),
                        DriveConstants.IOControlsConstants.kDriveDeadband),
                    MathUtil.applyDeadband(
                        -m_xboxController.getRightX(),
                        DriveConstants.IOControlsConstants.kDriveDeadband),
                    false,
                    true),
            m_drive));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
