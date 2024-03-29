// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPosePPCommand extends ParallelCommandGroup {

  public DriveToPosePPCommand(SwerveSubsystem drive, Pose2d desiredPose) {

    setName("DriveToPosePPCommand");

    addCommands(drive.driveToPose(desiredPose));
  }

  public DriveToPosePPCommand(SwerveSubsystem drive, Supplier<Pose2d> desiredPose) {
    setName("DriveToPosePPCommand");
    addCommands(
        new InstantCommand(
            () -> {
              drive.driveToPose(desiredPose.get()).schedule();
            }));
  }
}
