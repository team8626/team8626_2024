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
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.util.Units;

public class RobotContainer {

  Dashboard m_dashboard;

  //DriveSubsystem m_drive = new DriveSubsystem();
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                          "swerve"));

  XboxController m_xboxController = new XboxController(0);

  public RobotContainer() {

    configureBindings();
    configureDefaultCommands();

    //m_dashboard = new Dashboard(m_drive);

    // TODO: Get rid, only for testing drive to pose
    // m_drive.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }
  private void configureBindings() {}
  private void configureDefaultCommands() {

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(m_drivebase,
                                                                   () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(m_xboxController.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(-m_xboxController.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   m_xboxController::getBButtonPressed,
                                                                   m_xboxController::getAButtonPressed,
                                                                   m_xboxController::getXButtonPressed,
                                                                   m_xboxController::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
        () -> m_xboxController.getRightX(),
        () -> m_xboxController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = m_drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
        () -> -m_xboxController.getRawAxis(4));

    Command driveFieldOrientedDirectAngleSim = m_drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-m_xboxController.getLeftY(), 0.1),
        () -> MathUtil.applyDeadband(-m_xboxController.getLeftX(), 0.1),
        () -> -m_xboxController.getRawAxis(4));

    m_drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
      //  !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);





        //   m_drive.setDefaultCommand(
        // // The left stick controls translation of the robot.
        // // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        //     () -> m_drive.drive(
        //         MathUtil.applyDeadband(-m_xboxController.getLeftY(), DriveConstants.IOControlsConstants.kDriveDeadband),
        //         MathUtil.applyDeadband(-m_xboxController.getLeftX(), DriveConstants.IOControlsConstants.kDriveDeadband),
        //         MathUtil.applyDeadband(-m_xboxController.getRightX(), DriveConstants.IOControlsConstants.kDriveDeadband),
        //         false,
        //         true),
        //     m_drive));     
  }
  

  public Command getAutonomousCommand() {
    // return m_drive.getDriveToPoseCommand(1, 1, 150);
    // return m_drive.getDriveToPoseCommand(0, 0, 45);
    return null;
  }
}
