// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoseTrajPIDCommand extends SequentialCommandGroup {

  public DriveToPoseTrajPIDCommand(SwerveSubsystem drive, Pose2d desiredPose, boolean lockPose) {

    addCommands(
        new DriveToPosePPCommand(drive, desiredPose),
        new TurnToAngleCommand(drive, desiredPose.getRotation(), true),
        new InstantCommand(() -> drive.lock()).onlyIf(() -> lockPose));

    setName("Drive To Pose Trajectory and PID Command");
  }
}
