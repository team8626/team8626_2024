// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoseCommand extends ParallelCommandGroup {
  SwerveSubsystem m_drive;

  Command m_driveToPoseCommand;

  public DriveToPoseCommand(SwerveSubsystem drive, Pose2d desiredPose) {
    m_drive = drive;

    setName("Drive To Pose Command");
    addRequirements(m_drive);
    m_driveToPoseCommand = m_drive.driveToPose(desiredPose);
    addCommands(m_driveToPoseCommand);
  }
  // TODO: Once new PID DriveToPose is visible to this branch current pose supplier will work, add
  // positive 1 meter current pose X
  public DriveToPoseCommand(SwerveSubsystem drive, Supplier<Pose2d> currentPoseRunnable) {
    m_drive = drive;

    setName("Drive To Pose Command");
    addRequirements(m_drive);
    m_driveToPoseCommand = m_drive.driveToPose(currentPoseRunnable.get());
    addCommands(m_driveToPoseCommand);
  }
}
